import numpy as np

def getA_k(phi_k: float, # roll angle at time step k
            u1_k: float, # thrust force at time step k
            m: float,    # mass of the quadrotor
            dt: float):  # sample time 
    A_k = np.array([
        [1, 0,                                   0,       dt,       0,       0],
        [0, 1,                                   0,        0,      dt,       0],
        [0, 0,                                   1,        0,       0,      dt],
        [0, 0,      -u1_k / m * np.cos(phi_k) * dt,        1,       0,       0],
        [0, 0,      -u1_k / m * np.sin(phi_k) * dt,        0,       1,       0],
        [0, 0,                                   0,        0,       0,       1]
    ])
    
    return A_k

def getB_k(phi_k: float, # roll angle at time step k
            Ixx: float,
            m: float,
            dt: float):
    B_k = np.array([
        [0,                       0],
        [0,                       0],
        [0,                       0],
        [-np.sin(phi_k) / m * dt, 0],
        [ np.cos(phi_k) / m * dt, 0],
        [0,                dt / Ixx]
    ])

    return B_k