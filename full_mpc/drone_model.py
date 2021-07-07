from casadi import *

def drone_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Translational_drone"

    # Quadrotor intrinsic parameters
    # m = 1.0  # kg
    # J = np.array([.03, .03, .06])  # N m s^2 = kg m^2

    # Length of motor to CoG segment
    # length = 0.47 / 2  # m

    # crazyflie 2.0 parameters
    m =  0.033 # m=27g
    J = np.array([1.657171e-05, 1.657171e-05, 2.9261652e-05])
    length = 0.046


    # constants
    g = 9.81 # m/s^2

    ## CasAdi Model
    # set up states and controls
    px = MX.sym("px")
    py = MX.sym("py")
    pz = MX.sym("pz")
    qw = MX.sym("qw")
    qx = MX.sym("qx")
    qy = MX.sym("qy")
    qz = MX.sym("qz")
    vx = MX.sym("vx")
    vy = MX.sym("vy")
    vz = MX.sym("vz")
    x  = vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz)

    # controls
    T  = MX.sym("T")
    wx = MX.sym("qw")
    wy = MX.sym("qx")
    wz = MX.sym("qy")
    u = vertcat(T, wx, wy, wz)

    # xdot
    pxdot = MX.sym("pxdot")
    pydot = MX.sym("pydot")
    pzdot = MX.sym("pzdot")
    qwdot = MX.sym("qwdot")
    qxdot = MX.sym("qxdot")
    qydot = MX.sym("qydot")
    qzdot = MX.sym("qzdot")
    vxdot = MX.sym("vxdot")
    vydot = MX.sym("vydot")
    vzdot = MX.sym("vzdot")
    xdot  = vertcat(pxdot, pydot, pzdot, qwdot, qxdot, qydot, qzdot, vxdot, vydot, vzdot)

    # algebraic variables 
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    f_expl = vertcat(
        vx,
        vy,
        vz,
        0.5 * ( - wx * qx - wy * qy - wz * qz),
        0.5 * (   wx * qw + wz * qy - wy * qz),
        0.5 * (   wy * qw - wz * qx + wx * qz),
        0.5 * (   wz * qw + wy * qx - wx * qy),
        2 * ( qw * qy + qx * qz ) * T / m,
        2 * ( qy * qz - qw * qx ) * T / m,
        ( ( 1 - 2 * qx * qx - 2 * qy * qy ) * T ) / m - g 
    )
    
    model.phi_min = -80 * np.pi / 180
    model.phi_max =  80 * np.pi / 180

    model.theta_min = -80 * np.pi / 180
    model.theta_max =  80 * np.pi / 180

    # input bounds
    model.thrust_min = 0
    model.thrust_max = 0.9 * ((46e-3 * g)) # 90 % of max_thrust (max_thrust = 57g)

    model.torque_max = 1 / 2 * model.thrust_max * length # divided by 2 since we only have 2 propellers in a planar quadrotor
    model.torque_max = 0.1 * model.torque_max # keeping 10% margin for steering torque. This is done because the torque_max 
                                              # is the maximum torque that can be given around any one axis. But, we are going to
                                              # limit the torque greatly.
    model.torque_min = - model.torque_max

    # define initial condition
    model.x0 = np.array([0.0, 0.0, 1.595, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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