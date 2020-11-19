from math import cos,sin,pi
import numpy as np

class Motor:
    def __init__(self,pos_x,pos_y):
        self.x = pos_x
        self.y = pos_y
        self.z = 0
        self.thrust = 0

    def speed_up(self):
        self.thrust+=1
    
    def slow_down(self):
        self.thrust-=1
    
    def get_torque(self):
        force_vector = [0,0,self.thrust]
        radius_vector = [self.x,self.y,0]
        torque = -1*np.cross(force_vector,radius_vector)
        return torque

    def set_motor_specs(self,mass,radius,height):
        self.mass = mass
        self.radius = radius
        self.height = height

    def get_moments(self):
        Ix_about_motor = (1/4)*self.mass*self.radius*self.radius + (1/3)*self.mass*self.height*self.height
        Iy_about_motor = Ix_about_motor
        Iz_about_motor = (1/2)*self.mass*self.radius*self.radius

        return [Ix_about_motor + self.mass*self.y*self.y,Iy_about_motor + self.mass*self.x*self.x,Iz_about_motor + self.mass*(self.x*self.x + self.y*self.y)]

class Drone:
    def __init__(self,base_width,base_length,arm_length,arm_angle):
        arm_angle = pi*arm_angle/180
        self.bw = base_width
        self.bl = base_length

        x = base_width/2 + arm_length*cos(arm_angle)
        y = base_length/2 + arm_length*sin(arm_angle)
        self.motor1 = Motor(x,y)
        self.motor2 = Motor(x,-y)
        self.motor3 = Motor(-x,-y)
        self.motor4 = Motor(-x,y)
        self.motors = [self.motor1,self.motor2,self.motor3,self.motor4]
        for m in self.motors:
            m.set_motor_specs(36/1000,27.5/1000,21/1000)
    
    def set_base_mass(self,base_mass):
        self.base_mass = base_mass

    def set_base_thickness(self,thickness):
        self.base_thickness = thickness
    
    def get_effective_torque(self):
        torques = [m.get_torque() for m in self.motors]
        torque_x = sum([v[0] for v in torques])
        torque_y = sum([v[1] for v in torques])
        torque_z = sum([v[2] for v in torques])
        return[torque_x,torque_y,torque_z]

    def get_effective_moment(self):
        base_moment_x = (1/12)*self.base_mass*(self.bl*self.bl + self.base_thickness*self.base_thickness)
        base_moment_y = (1/12)*self.base_mass*(self.bw*self.bw + self.base_thickness*self.base_thickness)
        base_moment_z = (1/12)*self.base_mass*(self.bw*self.bw + self.bl*self.bl)
        moments = [m.get_moments() for m in self.motors]
        mx = sum([m[0] for m in moments])
        my = sum([m[1] for m in moments])
        mz = sum([m[2] for m in moments])
        return [mx + base_moment_x,my + base_moment_y,mz+base_moment_z]

    def get_alpha(self):
        tau = self.get_effective_torque()
        Im = self.get_effective_moment()
        return [tau[i]/Im[i] for i in range(3)]

    def get_effective_force(self):
        return np.array([0,0,sum([m.thrust for m in self.motors])])

    def get_accel(self):
        return self.get_effective_force()/(self.base_mass + 4*self.motor1.mass)

    def test(self):
        self.motor1.speed_up()
        # self.motor2.speed_up()
        # self.motor3.speed_up()
        # self.motor4.speed_up()
        # print(self.get_effective_torque())
        # print(self.get_effective_moment())
        print(self.get_alpha())
        print(self.get_accel())

if __name__ == '__main__':
    d = Drone(0.044,.1,.08,30)
    d.set_base_mass(36/1000)
    d.set_base_thickness(12.4/1000)
    d.test()
    