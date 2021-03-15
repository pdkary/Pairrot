import numpy as np
from math import cos,sin,pi,atan2


def get_xy_planar_transformation_matrix(x,y,z,theta):
    return np.array([[cos(theta),-sin(theta),0,x],[sin(theta),cos(theta),0,y],[0,0,1,z],[0,0,0,1]])

def get_pos(mat):
    ctheta = mat[0][0]
    stheta = mat[1][0]
    theta = atan2(stheta,ctheta)*180/pi
    pos_x = mat[0][3]
    pos_y = mat[1][3]
    pos_z = mat[2][3]
    return (pos_x,pos_y,pos_z,theta)

if __name__ == '__main__':
    origin_to_tracker = get_matrix(0,0,0,pi/2)
    origin_to_drone = get_matrix(1,1,0,0)

    drone_to_origin = np.linalg.inv(origin_to_drone)


    print(get_pos(np.matmul(drone_to_origin,origin_to_tracker)))


    