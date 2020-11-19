from dronekit import LocationGlobalRelative, LocationGlobal
import math
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;

class Tracker:
    speed = 1
    def __init__(self,position: LocationGlobalRelative,orientation:float,distance,theta,phi):
        self.position = position
        self.orientation = orientation
        self.distance = distance
        self.theta = theta
        self.phi = phi
        self.move_count = 0;

    def init_zig_zag(self,angle,zag_steps):
        self.zag_steps = zag_steps
        x_hat,y_hat = [math.cos(angle),math.sin(angle)]
        self.direction = [x_hat,y_hat]
        self.perp_cw = [y_hat,-x_hat]
        self.perp_ccw = [-y_hat,x_hat]
    
    def update_tracker_position(self,zag_multiplier=1):
        self.move_count+=1
        direction = self.perp_cw
        k = self.move_count*2/self.zag_steps
        if int(k) == k and k % 2 !=0:
            direction = self.perp_cw if direction==self.perp_ccw else self.perp_ccw
            
        dx = self.direction[0] + zag_multiplier*direction[0]
        dy = self.direction[1] + zag_multiplier*direction[1]
        self.position = get_location_metres(self.position,dx*Tracker.speed,dy*Tracker.speed)
    
    def get_drone_location(self):
        dx = self.distance*math.cos(self.phi)*math.cos(self.theta+self.orientation)
        dy = self.distance*math.cos(self.phi)*math.sin(self.theta+self.orientation)
        dz = self.distance*math.sin(self.phi)
        dp = get_location_metres(self.position,dy,dx)
        dp.alt = dp.alt + dz
        return dp


    