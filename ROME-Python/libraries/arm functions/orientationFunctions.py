import numpy as np
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
    
    # [:,1] gets row vector
    # [1,:] gets column vector
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



