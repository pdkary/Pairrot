from dronekit import LocationLocal
from math import cos, sin, sqrt, pi
from random import gauss
import numpy as np
from matplotlib import pyplot as plt


class CircleDataService:
    def __init__(self, T: float, R: float, sigma_P: float, sigma_O: float):
        self.T = T  # period between pings (completes in 60 cycles)
        self.R = R  # flight radius
        self.sigma_P = sigma_P  # stdev of the position sensor (meters)
        self.sigma_O = sigma_O  # stdev of the orientation sensor (radians)

    def set_initial(self, starting_position: LocationLocal, starting_orientation: float):
        self.position = starting_position
        self.orientation = starting_orientation

    def update(self):
        theta = 2*pi/60
        self.orientation += theta
        self.position.north = self.R*sin(self.orientation)
        self.position.east = self.R*cos(self.orientation)

    def read_position(self):
        noise = np.random.normal(0, self.sigma_P, 3)
        return LocationLocal(self.position.north + noise[0], self.position.east+noise[1], self.position.down+noise[2])

    def read_position_noiseless(self):
        return self.position

    def read_orientation(self):
        noise = np.random.normal(0, self.sigma_O, 1)
        return self.orientation + noise[0]
    
    def read_orientation_noiseless(self):
        return self.orientation



