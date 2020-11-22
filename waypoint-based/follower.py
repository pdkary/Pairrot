import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import pi,cos,sin
from random import gauss
from fake_tracker import Tracker, get_location_metres
from pymavlink import mavutil
import csv

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

class Follower:
    def __init__(self,home_pos: LocationGlobalRelative,distance:float,theta:float,phi:float):
        self.distance = distance
        self.theta = theta
        self.phi = phi
        self.home_poos = home_pos
        self.vehicle = connect(args.connect,wait_ready=True)
        self.tracker = Tracker(home_pos,0)
        self.var = 0
        self.flight_data = []

    def set_noise_variance(self,var):
        self.var = var

    def takeoff(self):
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        c = True
        while(c):
            print("Vehicle is in mode: {}, please set to GUIDED".format(self.vehicle.mode))
            break_loop = input("Continue? (y/n)")
            if break_loop == "y":
                break

        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        alt = self.get_desired_location().alt
        print("Taking off to {}".format(alt));
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

        self.append_data()

    def set_rtl(self):
        print("Landing")  
        msg = self.vehicle.message_factory.command_long_encode(
            0,0,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,0,0,0,0,0,0
        )
        self.vehicle.send_mavlink(msg)
    
    def get_desired_location(self):
        dx = self.distance*cos(self.phi)*cos(self.theta + self.tracker.orientation)
        dy = self.distance*cos(self.phi)*sin(self.theta+self.tracker.orientation)
        dz = self.distance*sin(self.phi)

        tracker_pos = self.tracker.position
        tracker_pos.lat += gauss(0,self.var)
        tracker_pos.lon += gauss(0,self.var)
        tracker_pos.alt += gauss(0,self.var)
        dp = get_location_metres(tracker_pos,dy,dx)
        dp.alt += dz
        return dp

    def go_to_tracker(self,sleep_time):
        tracking_pos = self.get_desired_location()
        self.vehicle.simple_goto(tracking_pos)
        time.sleep(sleep_time)

    def follow(self,steps):
        for i in range(steps):
            print("step {}".format(i))
            d = self.tracker.update_tracker_position(1)
            self.go_to_tracker(d/Tracker.speed)
            self.append_data()
    
    def append_data(self):
        data = {
                "track_lat":self.tracker.position.lat,
                "vehicle_lat":self.vehicle.location.global_relative_frame.lat,
                "track_lon":self.tracker.position.lon,
                "vehicle_lon":self.vehicle.location.global_relative_frame.lon,
                "track_alt":self.tracker.position.alt,
                "vehicle_alt":self.vehicle.location.global_relative_frame.alt
            }
        self.flight_data.append(data)

    def write_to_csv(self,csv_file):
        flight_data_columns = ["track_lat","vehicle_lat","track_lon","vehicle_lon","track_alt","vehicle_alt"]
        try:
            with open(csv_file,'w') as csvfile:
                writer = csv.DictWriter(csvfile,fieldnames=flight_data_columns)
                writer.writeheader()
                for data in self.flight_data:
                    writer.writerow(data)
        except IOError:
            print("I/O Error")

home_pos = LocationGlobalRelative(43.257090,-79.936480,3)

if __name__ == '__main__':
    f = Follower(home_pos,2,0,pi/2)
    f.set_noise_variance(.33/1000)
    f.tracker.set_circle(1)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("circle_10.csv")
    