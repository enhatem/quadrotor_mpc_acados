from extended_kalman_filter.setup_ekf import setup_ekf
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

from acados_settings import acados_settings
from extended_kalman_filter.ekf import *
from extended_kalman_filter.jacobians import *
from plotFnc import *
from utils import *
from trajectory import *

# mpc and simulation parameters
Tf = 1        # prediction horizon
N  = 100       # number of discretization steps
Ts = Tf / N   # sampling time[s]

T_hover = 1     # hovering time[s]
T_traj  = 20.00 # trajectory time[s]

T = T_hover + T_traj # total simulation time
T = 4

# constants
g = 9.81     # m/s^2

# bounds on phi
bound_on_phi = False

# bounds on y and z
bound_on_y_z = False

# measurement noise bool
noisy_measurement = False

# input noise bool
noisy_input = False

# extended kalman filter bool
extended_kalman_filter = True

# generate circulare trajectory with velocties
traj_with_vel = False

# single reference point with phi = 2 * pi
ref_point = False

# import trajectory with positions and velocities and inputs
import_trajectory = True

# use acados integrator (if False, numerical integration is used instead):
use_acados_integrator = True

# bool to save measurements and inputs as .csv files
save_data = True

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(
    Ts, Tf, N, bound_on_phi, bound_on_y_z)

# quadrotor parameters
m = model.params.m
Ixx = model.params.J[0]

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
N_hover = int (T_hover * N / Tf)
Nsim = int(T * N / Tf)

# initialize data structs
predX = np.ndarray((Nsim+1, nx))
simX  = np.ndarray((Nsim+1, nx))
simU  = np.ndarray((Nsim,   nu))
tot_comp_sum = 0
tcomp_max = 0

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent

# creating or extracting trajectory
if ref_point == False and import_trajectory == False:
    # creating a reference trajectory
    show_ref_traj = False
    radius = 1  # m
    freq = 8 * np.pi/10  # frequency

    if traj_with_vel == False:
        y, z = trajectory_generator2D(xcurrent, N_hover, Nsim, N, radius, show_ref_traj)
        ref_traj = np.stack((y, z), 1)
            # adding the hovering position to the beginning of the trajectory
        # ref_traj = add_hover(ref_traj)
    else:
        y, z, vy, vz = trajectory_generotaor2D_with_vel(
            xcurrent, N_hover, model, radius, freq, T_traj, Tf, Ts)
        ref_traj = np.stack((y, z, vy, vz), 1)

elif ref_point == False and import_trajectory == True:
    T, ref_traj, ref_U = readTrajectory(T_hover, N)
    Nsim = int((T-1) * N / Tf)
    predX = np.ndarray((Nsim+1, nx))
    simX  = np.ndarray((Nsim+1, nx))
    simU  = np.ndarray((Nsim,   nu))
    simX[0, :] = xcurrent

# setup the extended kalman filter
if extended_kalman_filter == True:
    ekf, C, D, Q_alpha, Q_beta, Q_gamma = setup_ekf(
        Ts, m, Ixx, xcurrent, nx, nu)
    MEAS_EVERY_STEPS = 1 # number of steps until a new measurement is taken
    states = []
    pred = []
    covs = []
    meas_xs = []

# elif extended_kalman_filter == True and noisy_measurement == False:
#     sys.exit("The extended Kalman filter must run with a noisy input")

# set the seed for the random variables (if noisy measurement and noisy input are applied)
np.random.seed(20)

# closed loop
for i in range(Nsim):

    # updating references
    if ref_point == False and import_trajectory == False:
        if traj_with_vel == False:
            for j in range(N):
                yref = np.array([y[i+j], z[i+j], 0.0, 0.0, 0.0,
                                0.0, model.params.m * g, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([y[i+N], z[i+N], 0.0, 0.0, 0.0, 0.0])
            acados_solver.set(N, "yref", yref_N)
        else:
            for j in range(N):
                yref = np.array([y[i+j], z[i+j], 0.0, vy[i+j],
                                vz[i+j], 0.0, model.params.m * g, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([y[i+N], z[i+N], 0.0, vy[i+j], vz[i+j], 0.0])
            acados_solver.set(N, "yref", yref_N)

    elif ref_point == True and import_trajectory == False:
        for j in range(N):
            yref = np.array([2.0, 2.0, 0.0 * np.pi, 0.0, 0.0,
                            0.0, model.params.m * g, 0.0])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([2.0, 2.0, 0.0 * np.pi, 0.0, 0.0, 0.0])
        acados_solver.set(N, "yref", yref_N)

    elif ref_point == False and import_trajectory == True:
        # if i == Nsim-5:
        #     print(f'i={i}')
        
        for j in range(N):
            y       = ref_traj[i+j, 0]
            z       = ref_traj[i+j, 1]
            phi     = ref_traj[i+j, 2]
            vy      = ref_traj[i+j, 3]
            vz      = ref_traj[i+j, 4]
            phiDot  = ref_traj[i+j, 5]

            u1      = ref_U[i+j, 0]  # Thrust
            u2      = ref_U[i+j, 1]  # Torque

            yref    = np.array([y, z, phi, vy, vz, phiDot, u1, u2])
            acados_solver.set(j, "yref", yref)

        y_e         = ref_traj[i+N, 0]
        z_e         = ref_traj[i+N, 1]
        phi_e       = ref_traj[i+N, 2]
        vy_e        = ref_traj[i+N, 3]
        vz_e        = ref_traj[i+N, 4]
        phiDot_e    = ref_traj[i+N, 5]

        yref_N      = np.array([y_e, z_e, phi_e, vy_e, vz_e, phiDot_e])
        acados_solver.set(N, "yref", yref_N)

    # solve ocp for a fixed reference
    acados_solver.set(0, "lbx", xcurrent)
    acados_solver.set(0, "ubx", xcurrent)
    comp_time = time.time()
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    # manage timings
    elapsed = time.time() - comp_time
    tot_comp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution from acados_solver
    u0 = acados_solver.get(0, "u")

    if noisy_input == True:
        magnitude_u1 = Q_beta[0][0]  # magnitude of the input noise on thrust
        magnitude_u2 = Q_beta[1][1]  # magnitude of the input noise on torque

        # adding noise to the inputs
        T_noisy = u0[0] + np.array(np.random.normal(0, magnitude_u1))
        Tau_noisy = u0[1] + np.array(np.random.normal(0, magnitude_u2))

        # making sure that inputs are within bounds
        T_noisy = max(min(T_noisy, model.thrust_max), model.thrust_min)
        Tau_noisy = max(min(Tau_noisy, model.torque_max), model.torque_min)

        u0 = np.array([T_noisy, Tau_noisy])

    # storing results from acados solver
    simU[i, :] = u0

    # add measurement noise
    if noisy_measurement == True and extended_kalman_filter==False:
        xcurrent_sim = add_measurement_noise(xcurrent)
    elif noisy_measurement == True and extended_kalman_filter==True:
        xcurrent_sim = add_measurement_noise_with_kalman(xcurrent, Q_gamma)
        # xcurrent_sim = add_measurement_noise(xcurrent)
    else:
        xcurrent_sim = xcurrent

    if extended_kalman_filter == True:
        phi_k   = float (ekf.state[2])  # roll at time k

        # stacking the covariance matrix and the state estimation at each time instant
        covs.append(ekf.cov)
        states.append(ekf.state)
        
        # The measurement vector at each time instant
        meas_x = xcurrent_sim

        # calculating the Jacobians of the evolution model with respect to the state vector and the input vector
        T_k   = u0[0] # thrust at time step k
        Tau_k = u0[1] # torque at time step k 
        A_k = getA_k(phi_k, T_k, ekf._m, ekf._dt)
        B_k = getB_k(phi_k, Ixx, ekf._m, ekf._dt)

        # prediction phase
        ekf.predict(A = A_k, B = B_k, u = u0, Q_alpha = Q_alpha, Q_beta = Q_beta)
        pred.append(ekf.state)

        # correction phase
        if i != 0 and i % MEAS_EVERY_STEPS == 0:
            ekf.update(C = C, meas = meas_x, meas_variance = Q_gamma)
        meas_xs.append(meas_x)

    if use_acados_integrator == True:
        # simulate the system
        acados_integrator.set("x", xcurrent_sim)
        acados_integrator.set("u", u0)
        status = acados_integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))

        # get state
        xcurrent = acados_integrator.get("x")
    else:
        # integrate the accelerations
        delta_vel = integrateArray(getDynamics(m, Ixx, xcurrent_sim, u0), Ts)

        # integrate the velocities
        delta_X = integrateArray(delta_vel, Ts)

        # update the state
        xcurrent = updateState(xcurrent_sim, delta_X, delta_vel)

    # store state
    simX[i+1, :] = xcurrent

# save measurements and inputs as .csv files
if save_data == True and extended_kalman_filter == False:
    saveData(simX, simU)

if extended_kalman_filter == True:
    # converting lists to np arrays for plotting
    meas_xs = np.array(meas_xs)
    states = np.array(states)
    covs = np.array(covs)
    pred = np.array(pred)

# print the computation times100
print("Total computation time: {}".format(tot_comp_sum))
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))


if import_trajectory == False:
    t = np.arange(0, T, Ts)
else:
    t = np.arange(0, T-Tf ,Ts)

# plotting and RMSE

if extended_kalman_filter == True:

    if ref_point == False and import_trajectory == False:
        
        # root mean squared error on each axis
        rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim, :])
        rmse_y_kalman, rmse_z_kalman = rmseX_kalman(states, ref_traj[0:Nsim, :])

        # print the RMSE on each axis
        print("RMSE on y: {}".format(rmse_y))
        print("RMSE on z: {}".format(rmse_z))
        print("RMSE on y with EKF measurements: {}".format(rmse_y_kalman))
        print("RMSE on z with EKF measurements: {}".format(rmse_z_kalman))

        # check if circular trajectory is with velocity or not
        if traj_with_vel == False:  # circular trajectory is generated without velocities
            plotSim_kalman(simX, states, pred, ref_traj, Nsim, save=True)
            plotPos_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)
            plotVel_without_vy_vz_references_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)            
            plotSimU(t, simU, Nsim, save=True)
            plotErrors_without_vel_kalman(t, simX, states, ref_traj, Nsim, save=True)
        else:                       # circular trajectory is generated with velocities
            plotSim_kalman(simX, states, pred, ref_traj, Nsim, save=True)
            plotPos_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)
            plotVel_with_vy_vz_references_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)
            plotSimU(t, simU, Nsim, save=True)
            plotErrors_with_vel_kalman(t, simX, states, ref_traj, Nsim, save=True)

    if ref_point == True and import_trajectory == False:  # For single reference points
        plotSim_ref_point_kalman(simX, states, save=True)
        plotPos_ref_point_kalman(t, simX, states, save=True)
        plotVel_with_ref_pose_kalman(t,simX, states, covs, Nsim, save=True)            
        plotSimU(t, simU, Nsim, save=True)

    if ref_point == False and import_trajectory == True:  # For imported trajectories with velocities and inputs
        plotSim_kalman(simX, states, pred, ref_traj, Nsim, save=True)
        plotPos_with_imported_traj_kalman(t, simX, states, ref_traj, Nsim, save=True)
        plotVel_with_imported_traj_kalman(t, simX, states, ref_traj, Nsim, save=True)
        plotSimU_with_ref(t, simU, ref_U, Nsim, save=True)
        plotErrors_with_ref_kalman(t, simX, states, ref_traj, Nsim, save=True)
else:
    
    if ref_point == False and import_trajectory == False:
        
        # root mean squared error on each axis
        rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim, :])

        # print the RMSE on each axis
        print("RMSE on y: {}".format(rmse_y))
        print("RMSE on z: {}".format(rmse_z))

        # check if circular trajectory is with velocity or not
        if traj_with_vel == False:                      # circular trajectory is generated without velocities
            plotSim(simX, ref_traj, Nsim, save=True)
            plotPos(t, simX, ref_traj, Nsim, save=True)
            plotVel(t, simX, Nsim, save=True)            
            plotSimU(t, simU, Nsim, save=True)
            plotErrors_no_vel(t, simX, ref_traj, Nsim)
        else:                                           # circular trajectory is generated with velocities
            plotSim(simX, ref_traj, Nsim, save=True)
            plotPos(t, simX, ref_traj, Nsim, save=True)
            plotVel_with_vy_vz_references(t,simX, ref_traj, Nsim, save=True)
            plotSimU(t, simU, Nsim, save=True)
            plotErrors_with_vel(t, simX, ref_traj, Nsim, save=True)
    
    if ref_point == True and import_trajectory == False:  # For single reference points
        plotSim_ref_point(simX, save=True)
        plotPos_ref_point(t, simX, save=True)
        plotVel_with_ref_pose(t,simX, Nsim, save=True)   
        plotSimU(t, simU, Nsim, save=True)

    if ref_point == False and import_trajectory == True:  # For imported trajectories with velocities and inputs
        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos_with_imported_traj(t, simX, ref_traj, Nsim, save=True)
        plotVel_with_imported_traj(t, simX, ref_traj, Nsim, save=True)
        plotSimU_with_ref(t, simU, ref_U, Nsim, save=True)
        plotErrors_with_ref(t, simX, ref_traj, Nsim, save=True)
'''

# plotting and RMSE
if not ref_point and not import_trajectory:
    # check if circular trajectory is with velocity or not

    # root mean squared error on each axis
    rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim, :])

    # print the RMSE on each axis
    print("RMSE on y: {}".format(rmse_y))
    print("RMSE on z: {}".format(rmse_z))

    if extended_kalman_filter == True:
        # root mean squared error on each axis
        rmse_y_kalman, rmse_z_kalman = rmseX_kalman(states, ref_traj[0:Nsim, :])

        # print the RMSE on each axis
        print("RMSE on y with EKF measurements: {}".format(rmse_y_kalman))
        print("RMSE on z with EKF measurements: {}".format(rmse_z_kalman))

    if traj_with_vel == False:  # circular trajectory is generated without velocities
        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos(t, simX, ref_traj, Nsim, save=True)
        plotVel(t, simX, Nsim, save=True)
        plotSimU(t, simU, Nsim, save=True)
        plotErrors_no_vel(t, simX, ref_traj, Nsim)

    elif traj_with_vel == True and extended_kalman_filter == False:  # ciruclar trajectory is generated with velocities
        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos(t, simX, ref_traj, Nsim, save=True)
        plotVel_with_vy_vz_references(t, simX, ref_traj, Nsim, save=True)
        plotSimU(t, simU, Nsim, save=True)
        plotErrors_with_vel(t, simX, ref_traj, Nsim)

    elif traj_with_vel == True and extended_kalman_filter == True:  # ciruclar trajectory is generated with velocities
        plotSim_kalman(simX, states, pred, ref_traj, Nsim, save=True)
        plotPos_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)
        plotVel_with_vy_vz_references_kalman(t, simX, states, pred, covs, ref_traj, Nsim, save=True)
        plotSimU(t, simU, Nsim, save=True)
        plotErrors_with_vel_kalman(t, simX, states, ref_traj, Nsim)


if ref_point and not import_trajectory:  # For single reference points
    plotSim_ref_point(simX, save=True)
    plotPos_ref_point(t, simX, save=True)
    plotVel(t, simX, Nsim, save=True)
    plotSimU(t, simU, Nsim, save=True)


if extended_kalman_filter == False:
    if not ref_point and import_trajectory:  # For imported trajectories with velocities and inputs

        # root mean squared error on each axis
        rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim, :])

        # print the RMSE on each axis
        print("RMSE on y: {}".format(rmse_y))
        print("RMSE on z: {}".format(rmse_z))

        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos_with_ref(t, simX, ref_traj, Nsim, save=True)
        plotVel_with_ref(t, simX, ref_traj, Nsim, save=True)
        plotSimU_with_ref(t, simU, ref_U, Nsim, save=True)
        plotErrors_with_ref(t, simX, ref_traj, Nsim)
else:
    rmse_y_kalman, rmse_z_kalman = rmseX_kalman(states, ref_traj[0:Nsim, :])

    # print the RMSE on each axis
    print("RMSE on y with EKF measurements: {}".format(rmse_y_kalman))
    print("RMSE on z with EKF measurements: {}".format(rmse_z_kalman))
    
    if not ref_point and import_trajectory:  # For imported trajectories with velocities and inputs

        # root mean squared error on each axis
        rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim, :])

        # print the RMSE on each axis
        print("RMSE on y: {}".format(rmse_y))
        print("RMSE on z: {}".format(rmse_z))

        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos_with_ref(t, simX, ref_traj, Nsim, save=True)
        plotVel_with_ref(t, simX, ref_traj, Nsim, save=True)
        plotSimU_with_ref(t, simU, ref_U, Nsim, save=True)
        plotErrors_with_ref_kalman(t, simX, states, ref_traj, Nsim)
'''

plt.show()
