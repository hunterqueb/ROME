import numpy as np
# from scipy.spatial.transform import Rotation as R
# from numba import jit
# import time
# import random
# import spatialmath.base as tr

def Vec2Skew(v):
    # Function to generate the skew symmetric matrix corresponding to a vector
    Mx = np.asmatrix([0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0])

    return Mx

def getOrientErr(C_ref,C):
    pass

