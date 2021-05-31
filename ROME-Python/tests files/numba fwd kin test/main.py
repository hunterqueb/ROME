import numpy as np
from scipy.spatial.transform import Rotation as R
from numba import jit
import time
import random


# @jit(parallel=True, fastmath=True)
def DH(theta, alpha, d, a):
    # DH1 = [np.math.cosd(theta) - np.math.sind(theta)*np.math.cosd(alpha) np.math.sind(theta)*np.math.sind(alpha) a*np.math.cosd(theta)
    #        np.math.sind(theta) np.math.cosd(theta)*np.math.cosd(alpha) - np.math.cosd(theta)*np.math.sind(alpha) a*np.math.sind(theta)
    #        0           np.math.sind(alpha)             np.math.cosd(alpha)             d
    #        0 0 0 1]

    row1 = np.asfarray([np.math.cos(theta), -np.math.sin(theta)*np.math.cos(alpha), np.math.sin(theta)*np.math.sin(alpha), a*np.math.cos(theta)])
    row2 = np.asfarray([np.math.sin(theta), np.math.cos(theta)*np.math.cos(alpha), -np.math.cos(theta)*np.math.sin(alpha), a*np.math.sin(theta)])
    row3 = np.asfarray([0,np.math.sin(alpha),np.math.cos(alpha),d])
    row4 = np.asfarray([0,0,0,1])

    DHMatrix = np.matrix([row1,row2,row3,row4])

    return DHMatrix


def noAR2FKZYZ(theta0):

    PI = np.math.pi

    theta1 = theta0[0]
    theta2 = theta0[1]
    theta3 = theta0[2]
    theta4 = theta0[3]
    theta5 = theta0[4]
    theta6 = theta0[5]

    d1 = 169.77
    a1 = 64.2
    alpha1 = -90*PI/180
    d2 = 0
    a2 = 305
    alpha2 = 0
    d3 = 0
    a3 = 0
    alpha3 = 90*PI/180
    d4 = -222.63
    a4 = 0
    alpha4 = -90*PI/180
    d5 = 0
    a5 = 0
    alpha5 = 90*PI/180
    d6 = -36.25
    a6 = 0
    alpha6 = 0

    DH1 = DH(theta1, alpha1, d1, a1)
    DH2 = DH(theta2, alpha2, d2, a2)
    DH3 = DH(theta3, alpha3, d3, a3)
    DH4 = DH(theta4, alpha4, d4, a4)
    DH5 = DH(theta5, alpha5, d5, a5)
    DH6 = DH(theta6, alpha6, d6, a6)

    HT6 = DH1 * DH2 * DH3 * DH4 * DH5 * DH6

    pos = np.asarray([HT6[0, 3], HT6[1, 3], HT6[2, 3]])

    r = R.from_matrix(HT6[:3,:3])
    ori = r.as_euler('zyz')
    
    return np.concatenate((pos,ori),axis=None)


@jit(parallel=True, fastmath=True)
def AR2FKZYZ(theta0):

    PI = np.math.pi

    theta1 = theta0[0]
    theta2 = theta0[1]
    theta3 = theta0[2]
    theta4 = theta0[3]
    theta5 = theta0[4]
    theta6 = theta0[5]

    d1 = 169.77
    a1 = 64.2
    alpha1 = -90*PI/180
    d2 = 0
    a2 = 305
    alpha2 = 0
    d3 = 0
    a3 = 0
    alpha3 = 90*PI/180
    d4 = -222.63
    a4 = 0
    alpha4 = -90*PI/180
    d5 = 0
    a5 = 0
    alpha5 = 90*PI/180
    d6 = -36.25
    a6 = 0
    alpha6 = 0

    DH1 = DH(theta1, alpha1, d1, a1)
    DH2 = DH(theta2, alpha2, d2, a2)
    DH3 = DH(theta3, alpha3, d3, a3)
    DH4 = DH(theta4, alpha4, d4, a4)
    DH5 = DH(theta5, alpha5, d5, a5)
    DH6 = DH(theta6, alpha6, d6, a6)

    HT6 = DH1 * DH2 * DH3 * DH4 * DH5 * DH6

    pos = np.asarray([HT6[0, 3], HT6[1, 3], HT6[2, 3]])

    r = R.from_matrix(HT6[:3, :3])
    ori = r.as_euler('zyz')

    return np.concatenate((pos, ori), axis=None)


if __name__ == "__main__":
    theta0 = np.asarray([0, 0, 10, 0, 0, 0])
    state = AR2FKZYZ(theta0)

    start_time = time.time()

    for i in range(10):
        theta0 = np.asarray([random.randint(-10, 10), random.randint(-10, 10), random.randint(-10,10), random.randint(-10, 10), random.randint(-10, 10), random.randint(-10, 10)])
        state = AR2FKZYZ(theta0)

    end_time = time.time()

    print("numba time")
    print(end_time-start_time)

    start_time = time.time()

    for i in range(10):
        theta0 = np.asarray([random.randint(-10, 10), random.randint(-10, 10), random.randint(-10, 10),
                             random.randint(-10, 10), random.randint(-10, 10), random.randint(-10, 10)])
        state = noAR2FKZYZ(theta0)

    end_time = time.time()

    print("no numba time")
    print(end_time-start_time)
