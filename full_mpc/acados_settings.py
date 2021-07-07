from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import acados_template
from drone_model import drone_model
from acados_integrator import export_drone_integrator

import scipy.linalg
import numpy as np

def acados_settings(Ts, Tf, N):

    # create OCP object to formulate the optimization
    ocp = AcadosOcp()

    # export model
    model = drone_model()

    # constants
    g = 9.81 # m/s^2

    # define acados ODE 
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # dimensions 
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx 

    # discretization 
    ocp.dims.N = N
    
    # set cost 
    Q = np.eye(nx)
    Q[0][0] = 0e0  # weight of px
    Q[1][1] = 5e1  # weight of py
    Q[2][2] = 5e1  # weight of pz
    Q[3][3] = 1e0  # weight of qw
    Q[4][4] = 1e0  # weight of qx
    Q[5][5] = 1e0  # weight of qy
    Q[6][6] = 1e0  # weight of qz
    Q[7][7] = 0e0  # weight of vx
    Q[8][8] = 0e0  # weight of vy
    Q[9][9] = 0e0  # weight of vz

    R = np.eye(nu)
    R[0][0] = 1e0  # weight of Thrust
    R[1][1] = 0e0  # weight of wx
    R[2][2] = 1e0  # weight of wy
    R[3][3] = 1e0  # weight of wz

    Qe = np.eye(nx)
    Qe[0][0] = 0e0  # terminal weight of px
    Qe[1][1] = 5e1  # terminal weight of py
    Qe[2][2] = 5e1  # terminal weight of pz
    Qe[3][3] = 1e0  # terminal weight of qw
    Qe[4][4] = 1e0  # terminal weight of qx
    Qe[5][5] = 1e0  # terminal weight of qy
    Qe[6][6] = 1e0  # terminal weight of qz
    Qe[7][7] = 0e0  # terminal weight of vx
    Qe[8][8] = 0e0  # terminal weight of vy
    Qe[9][9] = 0e0  # terminal weight of vz

    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    
    ocp.cost.W   = scipy.linalg.block_diag(Q,R)
    ocp.cost.W_e = Qe

    Vx = np.zeros((ny,nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[-4:,-4:] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e
    
    # Initial reference trajectory (will be overwritten during the simulation)
    x_ref = np.array([1, 1, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref   = np.concatenate((x_ref, np.array([model.params.m * g, 0.0, 0.0, 0.0])))

    ocp.cost.yref_e = x_ref

    # set constraints on thrust and angular velocities
    ocp.constraints.lbu   = np.array([model.thrust_min, -4*np.pi, -4*np.pi, -4*np.pi])
    ocp.constraints.ubu   = np.array([model.thrust_max,  4*np.pi,  4*np.pi,  4*np.pi])
    ocp.constraints.idxbu = np.array([0,1,2,3])

    
    # ocp.constraints.lbx     = np.array([model.v_min, model.v_min, model.v_min])
    # ocp.constraints.ubx     = np.array([model.v_max, model.v_max, model.v_max])
    # ocp.constraints.idxbx   = np.array([7, 8, 9])

    '''
    ocp.constraints.lbx = np.array([-15.0, -15.0, -15.0]) # lower bounds on the velocity states
    ocp.constraints.ubx = np.array([ 15.0,  15.0,  15.0]) # upper bounds on the velocity states
    ocp.constraints.idxbx = np.array([3, 4, 5])
    '''

    # set initial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4

    # create ocp solver 
    acados_solver = AcadosOcpSolver(ocp, json_file=(model_ac.name + "_" + "acados_ocp.json"))

    acados_integrator = export_drone_integrator(Ts, model_ac)


    return model, acados_solver, acados_integrator