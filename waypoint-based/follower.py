import os
import time
from copy import deepcopy
from random import gauss
from pymavlink import mavutil
from mock_data.zig_zag_data_service import ZigZagDataService
from mock_data.circle_data_service import CircleDataService
from helpers.distance_helpers import get_location_metres, get_tranformation_delta, get_xy_planar_transformation_matrix
from follower_base import FollowerBase
import csv
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from math import pi, cos, sin, sqrt
import sys
import numpy as np
sys.path.insert(1, './mock_data/')
sys.path.insert(1, './helpers/')

class Follower(FollowerBase):
    def __init__(self, home_pos: LocationGlobalRelative, distance: float, theta: float, phi: float,connect_args):
        super().__init__(home_pos,distance,theta,phi,connect_args)
        self.horizon = 30
        self.previous_n_points = []
        self.flight_data = []

    def get_tracker_frame(self):
        tracker_pos = self.tracker.read_position()
        tracker_yaw = self.tracker.read_orientation()
        return get_xy_planar_transformation_matrix(tracker_pos.east, tracker_pos.north, tracker_pos.down, tracker_yaw)

    def get_drone_frame(self):
        drone_pos = self.vehicle.location.local_frame
        drone_yaw = self.vehicle.attitude.yaw
        return get_xy_planar_transformation_matrix(drone_pos.east, drone_pos.north, drone_pos.down, drone_yaw)

    def get_drone_state(self):
        tracker_yaw = self.tracker.read_orientation()
        ideal_drone_to_tracker = get_xy_planar_transformation_matrix(
            self.desired_relative_x, self.desired_relative_y, -self.desired_relative_z, tracker_yaw)

        origin_to_tracker = self.get_tracker_frame()
        origin_to_drone = self.get_drone_frame()

        tracker_to_origin = np.linalg.inv(origin_to_tracker)
        tracker_to_drone = np.linalg.inv(drone_to_tracker)

        ## Current position of the tracker (relative to origin)
        tx, ty, tz, ttheta = get_tranformation_delta(origin_to_tracker)

        ## Current position of the drone (relative to origin)
        dx, dy, dz, dtheta = get_tranformation_delta(drone_to_tracker)

        ## error
        ex, ey, ez, etheta = tx-dx,ty-dy,tz-dz,ttheta-dtheta

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

        ## we want to bring this bad boy to 0  

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

    def follow(self, num_iterations):
        for i in range(num_iterations):
            self.tracker.update()
            self.move_to_location()
            self.save_data()
            time.sleep(self.tracker.T)


parser = argparse.ArgumentParser(
    description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

home_pos = LocationGlobalRelative(43.257090, -79.936480, 3)

if __name__ == '__main__':
    f = Follower(home_pos, 5, 0, pi/2,args.connect)
    f.takeoff()
    f.follow(250)
    f.set_rtl()
    f.write_to_csv("zig_zag_v=1.csv")
