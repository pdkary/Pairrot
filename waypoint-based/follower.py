import os
import time
from copy import deepcopy
from random import gauss
from pymavlink import mavutil
from mock_data.zig_zag_data_service import ZigZagDataService
from mock_data.circle_data_service import CircleDataService
from helpers.distance_helpers import get_location_metres, get_tranformation_delta, get_xy_planar_transformation_matrix
import csv
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from math import pi, cos, sin, sqrt
import sys
import numpy as np
sys.path.insert(1, './mock_data/')
sys.path.insert(1, './helpers/')


parser = argparse.ArgumentParser(
    description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()


class Follower:
    def __init__(self, home_pos: LocationGlobalRelative, distance: float, theta: float, phi: float):
        self.distance = distance
        self.theta = theta
        self.phi = phi

        self.desired_relative_x = distance*cos(phi)*cos(theta)
        self.desired_relative_y = distance*cos(phi)*sin(theta)
        self.desired_relative_z = distance*sin(phi)

        self.desired_altitude = home_pos.alt + self.desired_relative_z

        self.home_pos = home_pos
        self.vehicle = connect(args.connect, wait_ready=True)

        self.tracker = ZigZagDataService(T=1, V=.5, sigma_P=0.5, sigma_O=0.2)
        self.tracker.set_initial(LocationLocal(0, 0, -home_pos.alt), 0,pi/2)

        self.horizon = 30
        self.previous_n_points = []
        self.flight_data = []

    def takeoff(self):
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        while self.vehicle.mode != 'GUIDED':
            print("Vehicle is in mode: {}, please set to GUIDED".format(
                self.vehicle.mode))
            time.sleep(3)

        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        alt = self.desired_altitude
        print("Taking off to {}".format(alt))
        self.vehicle.simple_takeoff(alt)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def set_rtl(self):
        print("Landing")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)

    def get_tracker_frame(self):
        tracker_pos = self.tracker.read_position()
        tracker_yaw = self.tracker.read_orientation()
        return get_xy_planar_transformation_matrix(tracker_pos.east, tracker_pos.north, tracker_pos.down, tracker_yaw)

    def get_drone_frame(self):
        drone_pos = self.vehicle.location.local_frame
        drone_yaw = self.vehicle.attitude.yaw
        return get_xy_planar_transformation_matrix(drone_pos.east, drone_pos.north, drone_pos.down, drone_yaw)

    def move_to_location(self):
        tracker_yaw = self.tracker.read_orientation()
        ideal_drone_to_tracker = get_xy_planar_transformation_matrix(
            self.desired_relative_x, self.desired_relative_y, -self.desired_relative_z, tracker_yaw)

        origin_to_tracker = self.get_tracker_frame()
        drone_to_tracker = self.get_drone_frame()

        tracker_to_origin = np.linalg.inv(origin_to_tracker)
        tracker_to_drone = np.linalg.inv(drone_to_tracker)

        ## Current position of the tracker (relative to origin)
        tx, ty, tz, ttheta = get_tranformation_delta(origin_to_tracker)

        ## Current position of the drone (relative to origin)
        dx, dy, dz, dtheta = get_tranformation_delta(drone_to_tracker)

        ## error
        ex, ey, ez, etheta = tx-dx,ty-dy,tz-dz,ttheta-dtheta  

        self.previous_n_points.insert(0, [tx, ty, tz])
        self.previous_n_points = self.previous_n_points[0:self.horizon]

        if len(self.previous_n_points) >= self.horizon:
            next_x, next_y, next_z = self.predict_next()
            origin_to_tracker = get_xy_planar_transformation_matrix(
                next_x, next_y, next_z, tracker_yaw)
            tracker_to_origin = np.linalg.inv(origin_to_tracker)

        ideal_drone_to_origin = np.matmul(
            ideal_drone_to_tracker, tracker_to_origin)

        px, py, pz, ptheta = get_tranformation_delta(ideal_drone_to_origin)
        offset_home = LocationGlobalRelative(
            self.home_pos.lat, self.home_pos.lon, self.desired_altitude)
        self.vehicle.simple_goto(get_location_metres(offset_home, -py,-px))

    def predict_next(self):
        n = len(self.previous_n_points)
        independant_var = [i for i in range(n)]
        x_poly = np.polyfit(independant_var, self.previous_n_points, 3)
        return x_poly[0]*(n**3) + x_poly[1]*(n**2) + x_poly[2]*n + x_poly[3]

    def save_data(self):
        tracker_pos = self.tracker.read_position_noiseless()
        tracker_pos_noisy = self.tracker.read_position()
        drone_pos = self.vehicle.location.local_frame

        tracker_heading = self.tracker.read_orientation_noiseless()
        tracker_heading_noisy = self.tracker.read_orientation()[0]
        drone_heading = self.vehicle.attitude.yaw

        self.flight_data.append({
            'tracker_N': tracker_pos.north,
            'tracker_E': tracker_pos.east,
            'tracker_D': tracker_pos.down,
            'tracker_heading': tracker_heading,
            'tracker_N_noisy': tracker_pos_noisy.north,
            'tracker_E_noisy': tracker_pos_noisy.east,
            'tracker_D_noisy': tracker_pos_noisy.down,
            'tracker_heading_noisy': tracker_heading_noisy,
            'drone_N': drone_pos.north,
            'drone_E': drone_pos.east,
            'drone_D': drone_pos.down,
            'drone_heading': drone_heading
        })

    def set_yaw(self, heading):
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def follow(self, steps):
        for i in range(steps):
            self.tracker.update()
            self.move_to_location()
            self.save_data()
            time.sleep(self.tracker.T)

    def write_to_csv(self, csv_file):
        flight_data_columns = ["tracker_N", "tracker_E", "tracker_D",
                               "tracker_heading", "tracker_N_noisy", 
                               "tracker_E_noisy", "tracker_D_noisy", 
                               "tracker_heading_noisy", "drone_N", 
                               "drone_E", "drone_D", "drone_heading"]
        try:
            with open(csv_file, 'w') as csvfile:
                writer = csv.DictWriter(
                    csvfile, fieldnames=flight_data_columns)
                writer.writeheader()
                for data in self.flight_data:
                    writer.writerow(data)
        except IOError:
            print("I/O Error")


home_pos = LocationGlobalRelative(43.257090, -79.936480, 3)

if __name__ == '__main__':
    f = Follower(home_pos, 5, 0, pi/2)
    f.takeoff()
    f.follow(250)
    f.set_rtl()
    f.write_to_csv("zig_zag_v=1.csv")
