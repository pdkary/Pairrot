from abc import ABC,abstractmethod
from pymavlink import mavutil
from mock_data.zig_zag_data_service import ZigZagDataService
from mock_data.circle_data_service import CircleDataService
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from math import pi, cos, sin, sqrt
import time
import sys
import csv

sys.path.insert(1, './mock_data/')
sys.path.insert(1, './helpers/')

class FollowerBase:
    def __init__(self, home_pos: LocationGlobalRelative, distance: float, theta: float, phi: float,connect_args):
        self.distance = distance
        self.theta = theta
        self.phi = phi

        self.desired_relative_x = distance*cos(phi)*cos(theta)
        self.desired_relative_y = distance*cos(phi)*sin(theta)
        self.desired_relative_z = distance*sin(phi)

        self.desired_altitude = home_pos.alt + self.desired_relative_z

        self.home_pos = home_pos
        self.vehicle = connect(connect_args, wait_ready=True)

        self.tracker = ZigZagDataService(T=1, V=.5, sigma_P=0.5, sigma_O=0.2)
        self.tracker.set_initial(LocationLocal(0, 0, -home_pos.alt), 0,pi/2)

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
    
    def set_ned_velocity(self,vx,vy,vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0
        )
        self.vehicle.send_mavlink(msg)

    def follow(self,num_iterations):
        pass

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