import numpy as np

def evolution(X, U, m, Ixx, dt):
    g = 9.81 # m/s^2

    y       = X[0] + X[3]*dt
    z       = X[1] + X[4]*dt
    phi     = X[2] + X[5]*dt
    vy      = X[3] + (    - U[0] / m ) * np.sin(X[2]) * dt
    vz      = X[4] + ( -g + U[0] / m ) * np.cos(X[2]) * dt
    phi_dot = X[5] + U[1] / Ixx * dt

    return np.array([y, z, phi, vy, vz, phi_dot]) 