from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from drone_model import drone_model
from acados_integrator import export_drone_integrator

import scipy.linalg
import numpy as np

def acados_settings(Ts, Tf, N, bound_on_phi, bound_on_y_z):

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
    Q[0][0] = 1e0   # weight of py
    Q[1][1] = 1e0  # weight of pz
    Q[2][2] = 1e0   # weight of phi
    Q[3][3] = 10e0   # weight of vy
    Q[4][4] = 10e0   # weight of vz
    Q[5][5] = 1e0   # weight of phidot

    R = np.eye(nu)
    R[0][0] = 1e0  # weight of Thrust
    R[1][1] = 1e0  # weight of Torque

    Qe = np.eye(nx)
    Qe[0][0] = 1e0   # weight of py
    Qe[1][1] = 1e0   # weight of pz
    Qe[2][2] = 1e0   # weight of phi
    Qe[3][3] = 1e0   # weight of vy
    Qe[4][4] = 1e0   # weight of vz
    Qe[5][5] = 1e0   # weight of phidot


    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    
    ocp.cost.W   = scipy.linalg.block_diag(Q,R)
    ocp.cost.W_e = Qe

    Vx = np.zeros((ny,nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[-2:,-2:] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e
    
    # Initial reference trajectory (will be overwritten during the simulation)
    x_ref = np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref   = np.concatenate((x_ref, np.array([model.params.m * g, 0.0])))

    ocp.cost.yref_e = x_ref

    # set constraints
    ocp.constraints.lbu     = np.array([model.thrust_min, -model.torque_max])
    ocp.constraints.ubu     = np.array([model.thrust_max, model.torque_max])
    ocp.constraints.idxbu   = np.array([0, 1])

    if bound_on_phi == False and bound_on_y_z==False:
        # constraints when no bounds are required
        ocp.constraints.lbx     = np.array([model.v_min, model.v_min])
        ocp.constraints.ubx     = np.array([model.v_max, model.v_max])
        ocp.constraints.idxbx   = np.array([3, 4])
    elif bound_on_phi == True and bound_on_y_z==False:
        # constraints when following a circular trajectory
        ocp.constraints.lbx     = np.array([model.phi_min, model.v_min, model.v_min])
        ocp.constraints.ubx     = np.array([model.phi_max, model.v_max, model.v_max])
        ocp.constraints.idxbx   = np.array([2, 3, 4])
    elif bound_on_phi == False and bound_on_y_z==True:
        # constraints when flipping
        ocp.constraints.lbx     = np.array([model.y_min, model.z_min, model.v_min, model.v_min])
        ocp.constraints.ubx     = np.array([model.y_max, model.z_max, model.v_max, model.v_max])
        ocp.constraints.idxbx   = np.array([0, 1, 3, 4])
    elif bound_on_phi == True and bound_on_y_z==True:
        # Not really used but it is coded for completeness
        ocp.constraints.lbx     = np.array([model.y_min, model.z_min, model.phi_min, model.v_min, model.v_min])
        ocp.constraints.ubx     = np.array([model.y_max, model.z_max, model.phi_max, model.v_max, model.v_max])
        ocp.constraints.idxbx   = np.array([0, 1, 2, 3, 4])
    

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