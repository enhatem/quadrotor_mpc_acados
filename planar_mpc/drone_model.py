import numpy as np
from casadi import *

def drone_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Planar_drone"

    # Quadrotor intrinsic parameters

    # crazyflie 2.0 parameters
    m = 0.027 # m=27g
    Ixx = 1.657171e-05
    Iyy = 1.657171e-05
    Izz = 2.9261652e-05
    J = np.array([Ixx, Iyy, Izz])
    length = 0.046

    # constants
    g = 9.81 # m/s^2

    ## CasAdi Model
    # set up states and controls
    #px = MX.sym("px")
    py      = MX.sym("py")
    pz      = MX.sym("pz")
    phi     = MX.sym("phi")
    vy      = MX.sym("vy")
    vz      = MX.sym("vz")
    phi_dot = MX.sym("phi_dot")
    x  = vertcat(py, pz, phi, vy, vz, phi_dot)

    # controls
    u1 = MX.sym("u1") # thurst (N)
    u2 = MX.sym("u2") # torque (N.m)
    u = vertcat(u1, u2)

    # xdot
    pydot  = MX.sym("py_dot")
    pzdot  = MX.sym("pz_dot")
    phidot = MX.sym("phi_dot") # same as phi_dot above (not sure if it was necessary to add it) 
    vydot  = MX.sym("vy_dot")
    vzdot  = MX.sym("vz_dot")
    phiddot = MX.sym("phi_ddot")
    
    xdot  = vertcat(pydot, pzdot, phidot, vydot, vzdot, phiddot)

    # algebraic variables 
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    f_expl = vertcat(
        vy,
        vz, 
        phi_dot,
        - u1 / m * np.sin(phi),
        - g + u1 / m * np.cos(phi), 
        u2 / Ixx
    )

    # input bounds
    model.throttle_min = 0
    model.throttle_max = 57e-3 * g # max_thrust = 57g

    # define initial condition
    model.x0 = np.array([1, 1, 0, 0, 0, 0]) # hovering at y=1, z=1 

    # define model struct
    params = types.SimpleNamespace()
    params.m = m
    params.J = J
    params.length = length
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    return model