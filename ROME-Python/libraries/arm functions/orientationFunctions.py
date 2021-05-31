import numpy as np
import armKinematics
import spatialmath.base as tr
# from scipy.spatial.transform import Rotation as R
# from numba import jit
# import time
# import random
# import spatialmath.base as tr

def Vec2Skew(v):
    # Function to generate the skew symmetric matrix corresponding to a vector
    Mx = np.asmatrix([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])

    return Mx

def getOrientErr(C_ref,C):
#   needs testing
#  
    # [:,1] gets column vector
    # [1,:] gets row vector
    nd = C_ref[:, 0]
    sd = C_ref[:, 1]
    ad = C_ref[:, 2]
    
    ne = C[:, 0]
    se = C[:, 1]
    ae = C[:, 2]
    
    S_nd = Vec2Skew(nd)
    S_sd = Vec2Skew(sd)
    S_ad = Vec2Skew(ad)
    S_ne = Vec2Skew(ne)
    S_se = Vec2Skew(se)
    S_ae = Vec2Skew(ae)


    eo = 0.5*(np.cross(ne, nd) + np.cross(se, sd) + np.cross(ae, ad))
    L = -0.5*(S_nd*S_ne + S_sd*S_se + S_ad*S_ae)

    return [eo, L]

def getPoseError(x_ref,theta_ref,q_init):
# needs testing

    armState = armKinematics.AR2FKZYZ(q_init)

    x_init = armState[0:3]
    theta_init = armState[3:6]

    ex_init = x_ref - x_init
    C_ref = tr.eul2r(theta_ref)
    C_init = tr.eul2r(theta_init)

    errors = getOrientErr(C_ref,C_init)

    err_init = [ex_init,errors[0]]

    return err_init


if __name__ == "__main__":
    
    theta0 = np.asarray([0, -1.396263401595464, 1.570796326794897, 0, 0, 0])

    armState = armKinematics.AR2FKZYZ(theta0)
    
    print(armState)
    print(armState[0:3])

    print(armState[3:6])
