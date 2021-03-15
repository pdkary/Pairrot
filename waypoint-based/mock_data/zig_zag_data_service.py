from dronekit import LocationLocal
from math import cos, sin, sqrt, pi, atan2
from random import gauss
import numpy as np


class ZigZagDataService:
    def __init__(self, T: float, V: float, sigma_P: float, sigma_O: float):
        self.T = T  # period between pings
        self.V = V  # drone velocity
        self.sigma_P = sigma_P  # stdev of the position sensor (meters)
        self.sigma_O = sigma_O  # stdev of the orientation sensor (radians)
        self.steps = 0

    def set_initial(self, starting_position: LocationLocal, starting_orientation: float, turn_angle: float):
        self.position = starting_position
        self.orientation = starting_orientation
        self.turn_angle = turn_angle
        self.turn(-turn_angle/2)

    def update(self):
        if self.steps % 5 == 0:
            if self.steps % 2 == 0:
                self.turn(self.turn_angle)
            else:
                self.turn(-self.turn_angle)
        self.steps += 1
        d = self.V*self.T
        self.position.north += d*sin(self.orientation)
        self.position.east += d*cos(self.orientation)

    def read_position(self):
        noise = np.random.normal(0, self.sigma_P, 3)
        return LocationLocal(self.position.north + noise[0], self.position.east+noise[1], self.position.down+noise[2])
    
    def read_position_noiseless(self):
        return self.position

    def read_orientation(self):
        noise = np.random.normal(0, self.sigma_P, 1)
        return self.orientation + noise

    def read_orientation_noiseless(self):
        return self.orientation
    
    def turn(self, rad):
        self.orientation += rad
    
