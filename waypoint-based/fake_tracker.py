from dronekit import LocationGlobalRelative
from math import cos,sin,sqrt,pi
from random import gauss
from distance_helpers import get_location_metres

class Tracker:
    speed = 1
    def __init__(self,position: LocationGlobalRelative,orientation:float):
        self.position = position
        self.orientation = orientation
        self.move_count = 0
        self.zig_zag = False
        self.circle = False

    def set_zig_zag(self,angle,zag_width):
        self.zig_zag = True
        self.circle = False
        self.zag_steps = zag_width
        x_hat,y_hat = [cos(angle),sin(angle)]
        self.direction = [x_hat,y_hat]
        self.perp_cw = [y_hat,-x_hat]
        self.perp_ccw = [-y_hat,x_hat]
        self.zig_direction = self.perp_cw

    def set_circle(self,radius):
        self.zig_zag=False
        self.circle=True
        self.radius = radius
        self.theta = 0
    
    def update_tracker_position(self,multiplier=1):
        if self.zig_zag:
            return self.update_tracker_zig_zag(multiplier)
        else:
            return self.update_tracker_circle()

    def update_tracker_zig_zag(self,zag_multiplier=1):
        self.move_count+=1
        k = self.move_count*2/self.zag_steps
        if int(k) == k and k % 2 !=0:
            self.zig_direction = self.perp_cw if self.zig_direction==self.perp_ccw else self.perp_ccw
            print("new direction: ",self.zig_direction)
            
        dx = self.direction[0] + zag_multiplier*self.zig_direction[0]
        dy = self.direction[1] + zag_multiplier*self.zig_direction[1]
        self.position = get_location_metres(self.position,dx,dy)
        d = sqrt(dx**2+dy**2)
        return d

    def update_tracker_circle(self):
        self.move_count+=1
        dx = -1*self.radius*sin(self.theta)
        dy = self.radius*cos(self.theta)
        print("({},{})\n".format(dx,dy))
        self.position = get_location_metres(self.position,dx,dy)
        self.theta+=Tracker.speed/self.radius
        d = sqrt(dx**2+dy**2)
        return d/self.radius
    