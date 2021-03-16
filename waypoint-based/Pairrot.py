import os
import time
from copy import deepcopy
from random import gauss
from pymavlink import mavutil
from mock_data.zig_zag_data_service import ZigZagDataService
from mock_data.circle_data_service import CircleDataService
from regulators.LQR import FiniteHorizonLQR
from follower_base import FollowerBase
from helpers.distance_helpers import get_location_metres, get_tranformation_delta, get_xy_planar_transformation_matrix
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from math import pi, cos, sin, sqrt
import sys
import numpy as np
sys.path.insert(1, './mock_data/')
sys.path.insert(1, './helpers/')
sys.path.insert(1,'./regulators/')

class Pairrot(FollowerBase):
    def __init__(self, home_pos: LocationGlobalRelative, distance: float, theta: float, phi: float,connect_args):
        super().__init__(home_pos,distance,theta,phi,connect_args)

        self.most_recent_tracker_frame = None
        self.previous_tracker_frame = None
        self.T = self.tracker.T
        self.A = np.array([[1,self.T,0,0,0,0],[0,0,0,0,0,0],[0,0,1,self.T,0,0],[0,0,0,0,0,0],[0,0,0,0,1,self.T],[0,0,0,0,0,0]])
        self.Bx = np.array([[0],[1],[0],[0],[0],[0]])
        self.By = np.array([[0],[0],[0],[1],[0],[0]])
        self.Bz = np.array([[0],[0],[0],[0],[0],[1]])

        ## high velocities should be expensive
        self.Q = np.array([[1,0,0,0,0,0],[0,100,0,0,0,0],[0,0,1,0,0,0],[0,0,0,100,0,0],[0,0,0,0,1,0],[0,0,0,0,0,100]])
        self.R = np.array([[100]])
        self.N = 15
        self.LQRx = FiniteHorizonLQR(self.A,self.Bx,self.Q,self.R,self.N)
        self.LQRy = FiniteHorizonLQR(self.A,self.By,self.Q,self.R,self.N)
        self.LQRz = FiniteHorizonLQR(self.A,self.Bz,self.Q,self.R,self.N)

        self.previous_ideal_position_queue = []
        self.previous_ideal_velocity_queue = []

    def get_drone_ideal_state(self):
        current_origin_to_tracker = self.get_tracker_frame()
        current_tracker_to_origin = np.linalg.inv(current_origin_to_tracker)

        if(self.previous_tracker_frame is None):
            self.previous_tracker_frame = current_origin_to_tracker

        previous_origin_to_tracker = self.previous_tracker_frame
        previous_tracker_to_origin = np.linalg.inv(previous_origin_to_tracker)
        ## ideal drone to tracker
        tracker_yaw = self.tracker.read_orientation()
        ideal_drone_to_tracker = get_xy_planar_transformation_matrix(self.desired_relative_x, self.desired_relative_y, -self.desired_relative_z, tracker_yaw)
        
        current_ideal_drone_to_origin = np.matmul(ideal_drone_to_tracker,current_tracker_to_origin)      
        previous_ideal_drone_to_origin = np.matmul(ideal_drone_to_tracker,previous_tracker_to_origin)

        cx,cy,cz,ctheta = get_tranformation_delta(current_ideal_drone_to_origin)
        px,py,pz,ptheta = get_tranformation_delta(previous_ideal_drone_to_origin)

        T = self.tracker.T
        ST = np.array([cx,(cx-px)/T,cy,(cy-py)/T,cz,(cz-pz)/T,ctheta,(ctheta-ptheta)/T])
        
        self.previous_ideal_position_queue.insert(0,[cx,cy,cz])
        self.previous_ideal_velocity_queue.insert(0,[(cx-px)/T,(cy-py)/T,(cz-pz)/T])
        self.previous_ideal_position_queue = self.previous_ideal_position_queue[0:self.N]
        self.previous_ideal_velocity_queue = self.previous_ideal_velocity_queue[0:self.N]
        
        if len(self.previous_ideal_position_queue) == self.N:
            n = len(self.previous_ideal_position_queue)
            independant = list(range(n))
            pos_poly = np.polyfit(independant,self.previous_ideal_position_queue,3)
            vel_poly = np.polyfit(independant,self.previous_ideal_velocity_queue,3)

            ## smoothed positions
            sx,sy,sz = pos_poly[0]*(n**3) + pos_poly[1]*(n**2) + pos_poly[2]*n + pos_poly[3]
            ## smoothed velocities
            svx,svy,svz = vel_poly[0]*(n**3) + vel_poly[1]*(n**2) + vel_poly[2]*n + vel_poly[3]
            ST = np.array([sx,svx,sy,svy,sz,svz,ctheta,(ctheta-ptheta)/T])
        
        return ST.transpose()

    def get_drone_state(self):
        drone_frame = self.get_drone_frame()
        currX,currY,currZ,currT = get_tranformation_delta(drone_frame)

        vX,vY,vZ = self.vehicle.velocity

        ST = np.array([currX,vX,currY,vY,currZ,vZ,currT,0])
        return ST.transpose()

    def get_state_error(self):
        ideal_state = self.get_drone_ideal_state()
        d_state = self.get_drone_state()
        return d_state - ideal_state
    
    def get_tracker_frame(self):
        tracker_pos = self.tracker.read_position()
        tracker_yaw = self.tracker.read_orientation()
        self.previous_tracker_frame = self.most_recent_tracker_frame
        self.most_recent_tracker_frame =  get_xy_planar_transformation_matrix(tracker_pos.east, tracker_pos.north, tracker_pos.down, tracker_yaw)
        return self.most_recent_tracker_frame

    def get_drone_frame(self):
        drone_pos = self.vehicle.location.local_frame
        drone_yaw = self.vehicle.attitude.yaw
        return get_xy_planar_transformation_matrix(drone_pos.east, drone_pos.north, drone_pos.down, drone_yaw)

    def follow(self,num_iterations):
        K = 0.1
        for i in range(num_iterations):
            self.tracker.update()
            self.save_data()
            Kx = self.LQRx.get_k()
            Ky = self.LQRy.get_k()
            Kz = self.LQRz.get_k()

            SE = self.get_state_error()
            SE_pos = SE[0:6]
            
            vx = K*Kx.dot(SE_pos)
            vy = K*Ky.dot(SE_pos)
            vz = K*Kz.dot(SE_pos)

            self.set_ned_velocity(-vy[0],-vx[0],vz[0])
            time.sleep(self.T)
            

parser = argparse.ArgumentParser(
    description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

home_pos = LocationGlobalRelative(43.257090, -79.936480, 3)

if __name__ == '__main__':
    f = Pairrot(home_pos, 5, 0, pi/2,args.connect)
    f.takeoff()
    f.follow(75)
    f.set_rtl()
    f.write_to_csv("LQR_test.csv")