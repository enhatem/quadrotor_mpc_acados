from casadi import *

def drone_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Translational_drone"

    # Quadrotor intrinsic parameters

    # crazyflie 2.0 parameters
    m = 0.027 # m=27g
    Ixx = 1.657171e-05
    Iyy = 1.657171e-05
    Izz = 2.9261652e-05
    J = np.array([Ixx, Iyy, Izz])
    length = 0.046 # distance from CoG to a rotor

    # constants
    g = 9.81 # m/s^2

    ## CasAdi Model
    # set up states and controls
    px = MX.sym("px")
    py = MX.sym("py")
    pz = MX.sym("pz")
    vx = MX.sym("vx")
    vy = MX.sym("vy")
    vz = MX.sym("vz")
    x  = vertcat(px, py, pz, vx, vy, vz)

    # controls
    T  = MX.sym("T")
    qw = MX.sym("qw")
    qx = MX.sym("qx")
    qy = MX.sym("qy")
    qz = MX.sym("qz")
    u = vertcat(T, qw, qx, qy, qz)

    # xdot
    pxdot = MX.sym("pxdot")
    pydot = MX.sym("pydot")
    pzdot = MX.sym("pzdot")
    vxdot = MX.sym("vxdot")
    vydot = MX.sym("vydot")
    vzdot = MX.sym("vzdot")
    xdot  = vertcat(pxdot, pydot, pzdot, vxdot, vydot, vzdot)

    # algebraic variables 
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    f_expl = vertcat(
        vx,
        vy,
        vz,
        2 * ( qw * qy + qx * qz ) * T,
        2 * ( qy * qz - qw * qx ) * T,
        ( 1 - 2 * qx * qx - 2 * qy * qy ) * T - g
    )

    # input bounds
    model.throttle_min = 0
    model.throttle_max = 57e-3 * g 
    
    # define initial condition
    model.x0 = np.array([1.0, 0.0, 1.0, 0.0, 0.0, 0.0])

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
