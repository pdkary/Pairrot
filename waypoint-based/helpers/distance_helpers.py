from math import cos,sin,pi,atan2
from dronekit import LocationGlobal,LocationGlobalRelative
import numpy as np

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*cos(pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/pi)
    newlon = original_location.lon + (dLon * 180/pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def get_tranformation_delta(mat):
    ctheta = mat[0][0]
    stheta = mat[1][0]
    theta = atan2(stheta,ctheta)*180/pi
    pos_x = mat[0][3]
    pos_y = mat[1][3]
    pos_z = mat[2][3]
    return (pos_x,pos_y,pos_z,theta)

def get_xy_planar_transformation_matrix(x,y,z,theta):
    return np.array([[cos(theta),-sin(theta),0,x],[sin(theta),cos(theta),0,y],[0,0,1,z],[0,0,0,1]])