import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from math import pi
from dronekit import LocationGlobalRelative
from follower import Follower
from data_evaluator import evaluate
from simple_plot import plot_data
from time import sleep

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

home_pos = LocationGlobalRelative(43.257090,-79.936480,3)

NOISE = 5e-6

def test_narrow_zig_zag_noiseless():
    f = Follower(home_pos,2,0,pi/2)
    f.tracker.set_zig_zag(-pi/2,2)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/zz_noiseless_2.csv")
    f.vehicle.close()

def test_narrow_zig_zag_noisy():
    f = Follower(home_pos,2,0,pi/2)
    f.set_noise_variance(NOISE)
    f.tracker.set_zig_zag(-pi/2,2)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/zz_noisy_2.csv")  
    f.vehicle.close()

def test_wide_zig_zag_noiseless():
    f = Follower(home_pos,2,0,pi/2)
    f.tracker.set_zig_zag(-pi/2,10)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/zz_noiseless_10.csv") 
    f.vehicle.close()

def test_wide_zig_zag_noisy():
    f = Follower(home_pos,2,0,pi/2)
    f.set_noise_variance(NOISE)
    f.tracker.set_zig_zag(-pi/2,10)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/zz_noisy_10.csv")
    f.vehicle.close()

def test_small_circle_noiseless():
    f = Follower(home_pos,2,0,pi/2)
    f.tracker.set_circle(1)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/circle_noiseless_1.csv")
    f.vehicle.close()

def test_small_circle_noisy():
    f = Follower(home_pos,2,0,pi/2)
    f.set_noise_variance(NOISE)
    f.tracker.set_circle(1)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/circle_noisy_1.csv")
    f.vehicle.close()

def test_wide_circle_noiseless():
    f = Follower(home_pos,2,0,pi/2)
    f.tracker.set_circle(5)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/circle_noiseless_5.csv")
    f.vehicle.close()

def test_wide_circle_noisy():
    f = Follower(home_pos,2,0,pi/2)
    f.set_noise_variance(NOISE)
    f.tracker.set_circle(5)
    f.takeoff()
    f.follow(50)
    f.set_rtl()
    f.write_to_csv("csv/circle_noisy_5.csv")
    f.vehicle.close()

if __name__ == '__main__':
    test_narrow_zig_zag_noiseless()
    test_narrow_zig_zag_noisy()
    test_wide_zig_zag_noiseless()
    test_wide_zig_zag_noisy()
    test_small_circle_noiseless()
    test_small_circle_noisy()
    test_wide_circle_noiseless()
    test_wide_circle_noisy()

    filenames = ["zz_noiseless_2.csv","zz_noisy_2.csv","zz_noiseless_10.csv","zz_noisy_10.csv","circle_noiseless_1.csv","circle_noisy_1.csv","circle_noiseless_5.csv","circle_noisy_5.csv"]
    for filename in filenames:
        rmse = evaluate("csv/"+filename)
        plot_data("csv/"+filename)
        print("{}: {}".format(filename,rmse))