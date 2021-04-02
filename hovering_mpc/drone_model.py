from casadi import *

def drone_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Hovering_drone"

    # Quadrotor intrinsic parameters
    m = 1.0  # kg
    J = np.array([.03, .03, .06])  # N m s^2 = kg m^2

    # Length of motor to CoG segment
    length = 0.47 / 2  # m

    # constants
    g = 9.81 # m/s^2

    ## CasAdi Model
    # set up states and controls
    pz = MX.sym("pz")
    vz = MX.sym("vz")
    x  = vertcat(pz,vz)

    # controls
    T = MX.sym("T")
    u = vertcat(T)

    # xdot
    pzdot = MX.sym("pzdot")
    vzdot = MX.sym("vzdot")
    xdot  = vertcat(pzdot, vzdot)

    # algebraic variables 
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    f_expl = vertcat(
        vz,
        - T / m + g
    )

    # input bounds
    model.throttle_min = 0
    model.throttle_max = 2*g

    # define initial condition
    model.x0 = np.array([-0.3,0])

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
