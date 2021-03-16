import csv
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sqrt,atan2,pi,cos,sin
import numpy as np
import pandas as pd
import sys

D = 5
THETA = 0
PHI = pi/2

desired_x = D*cos(PHI)*cos(THETA)
desired_y = D*cos(PHI)*sin(THETA)
desired_z = D*sin(PHI)

filename = sys.argv[1]
flight_data = pd.read_csv(filename)

fig = plt.figure(figsize=plt.figaspect(.2))
axes = [None,None,None,None]

axes[0] = fig.add_subplot(1,4,1)#,projection='3d')
axes[0].set_title("Nosiy Tracker and Drone position, R=5,θ=0,φ=90,σ=.5")
# axes[0].plot(flight_data["tracker_E_noisy"],flight_data["tracker_N_noisy"],-flight_data["tracker_D_noisy"],'C1',label="Tracker")
# axes[0].plot(flight_data["drone_E"],flight_data["drone_N"],-flight_data["drone_D"],"C2",label="Drone")
axes[0].plot(flight_data["tracker_E"],flight_data["tracker_N"],'C1',label="Tracker")
axes[0].plot(flight_data["drone_E"],flight_data["drone_N"],"C2",label="Drone")
# axes[0].legend()

exs = abs(desired_x - (flight_data["tracker_E"] - flight_data["drone_E"]))
eys = abs(desired_y - (flight_data["tracker_N"] - flight_data["drone_N"]))
ezs = abs(desired_z - (flight_data["tracker_D"] - flight_data["drone_D"]))

x_axis = [i for i in range(len(exs))]
axes[1] = fig.add_subplot(1,4,2)
axes[1].set_title("X-Error")
axes[1].plot(x_axis,exs)

axes[2] = fig.add_subplot(1,4,3)
axes[2].set_title("Y-Error")
axes[2].plot(x_axis,eys)

axes[3] = fig.add_subplot(1,4,4)
axes[3].set_title("Z-Error")
axes[3].plot(x_axis,ezs)

plt.savefig(filename.split(".")[0]+".png")
