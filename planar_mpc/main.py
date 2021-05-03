import time
import numpy as np
import matplotlib.pyplot as plt

from acados_settings import acados_settings
from plotFnc import *
from utils import *
from trajectory import *

# mpc and simulation parameters
Tf = 1        # prediction horizon
N = 100       # number of discretization steps
T = 20.00      # simulation time[s]
Ts = Tf / N   # sampling time[s]

# constants
g = 9.81     # m/s^2

# bounds on phi
bound_on_phi = True

# bounds on y and z
bound_on_y_z = False

# noise bool
noisy_measurement = False

# generate circulare trajectory with velocties
traj_with_vel = True

# single reference point with phi = 2 * pi
ref_point = False

# import trajectory with positions and velocities and inputs
import_trajectory = False

# use acados integrator (if False, numerical integration is used instead):
use_acados_integrator = True

# bool to save measurements and inputs as .csv files
save_data = True

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(Ts, Tf, N, bound_on_phi, bound_on_y_z)

# quadrotor parameters
m = model.params.m
Ixx = model.params.J[0]

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
predX = np.ndarray((Nsim+1, nx))
simX  = np.ndarray((Nsim+1, nx))
simU  = np.ndarray((Nsim,   nu))
tot_comp_sum = 0
tcomp_max = 0

if ref_point==False and import_trajectory==False:
    # creating a reference trajectory
    show_ref_traj = False
    radius  = 1 # m
    freq    = 6 * np.pi/10 # frequency

    if traj_with_vel == False:
        t, y, z = trajectory_generator2D(T, Tf, Nsim, N, radius, show_ref_traj)
        ref_traj = np.stack((y, z), 1)
    else:
        t, y, z, vy, vz = trajectory_generotaor2D_with_vel(model, radius, freq, T, Tf, Ts)
        ref_traj = np.stack((y, z, vy, vz), 1)
elif ref_point==False and import_trajectory==True:
    ref_traj, ref_U = readTrajectory(N)



# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent

# closed loop
for i in range(Nsim):
    
    # updating references
    
    if ref_point==False and import_trajectory==False:
        if traj_with_vel == False:
            for j in range(N):
                yref = np.array([y[i+j], z[i+j], 0.0, 0.0, 0.0, 0.0, model.params.m * g, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([y[i+N], z[i+N], 0.0, 0.0, 0.0, 0.0])
            acados_solver.set(N, "yref", yref_N)
        else:
            for j in range(N):
                yref = np.array([y[i+j], z[i+j], 0.0, vy[i+j], vz[i+j], 0.0, model.params.m * g, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([y[i+N], z[i+N], 0.0, vy[i+j], vz[i+j], 0.0])
            acados_solver.set(N, "yref", yref_N)
    
    elif ref_point==True and import_trajectory==False:
        for j in range(N):
            yref = np.array([1.3, 1.0, 2.0 * np.pi, 0.0, 0.0, 0.0, model.params.m * g, 0.0])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([1.3, 1.0, 2.0 * np.pi, 0.0, 0.0, 0.0])
        acados_solver.set(N, "yref", yref_N)
    
    elif ref_point==False and import_trajectory==True:
        for j in range(N):
            y       = ref_traj[i+j,0]
            z       = ref_traj[i+j,1]
            phi     = ref_traj[i+j,2]
            vy      = ref_traj[i+j,3]
            vz      = ref_traj[i+j,4]
            phiDot  = ref_traj[i+j,5]

            u1 = ref_U[i+j,0] # Thrust
            u2 = ref_U[i+j,1] # Torque

            yref = np.array([y, z, phi, vy, vz, phiDot, u1, u2])
            acados_solver.set(j, "yref", yref)
        
            y_e       = ref_traj[i+N,0]
            z_e       = ref_traj[i+N,1]
            phi_e     = ref_traj[i+N,2]
            vy_e      = ref_traj[i+N,3]
            vz_e      = ref_traj[i+N,4]
            phiDot_e  = ref_traj[i+N,5]

        yref_N = np.array([y_e, z_e, phi_e, vy_e, vz_e, phiDot_e])
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

    # storing results from acados solver
    simU[i, :] = u0

    # add measurement noise
    if noisy_measurement == True:
        xcurrent_sim = add_measurement_noise(xcurrent)
    else:
        xcurrent_sim = xcurrent

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
        delta_vel = integrateArray(getDynamics(m, Ixx,xcurrent_sim,u0), Ts)

        # integrate the velocities
        delta_X = integrateArray(delta_vel, Ts)

        # update the state
        xcurrent = updateState(xcurrent_sim,delta_X, delta_vel)

    # store state
    simX[i+1, :] = xcurrent

# save measurements and inputs as .csv files
if save_data == True:
    saveData(simX,simU)

# print the computation times100
print("Total computation time: {}".format(tot_comp_sum))
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))


if not ref_point and not import_trajectory:
    # check if circular trajectory is with velocity or not
    
    # root mean squared error on each axis
    rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim,:])

    # print the RMSE on each axis
    print("RMSE on y: {}".format(rmse_y))
    print("RMSE on z: {}".format(rmse_z))

    if traj_with_vel == False: # circular trajectory is generated without velocities
        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos(t,simX, ref_traj, Nsim, save=True)
        plotVel(t,simX, Nsim, save=True)
        plotSimU(t,simU, Nsim, save=True)
    
    elif traj_with_vel == True: # ciruclar trajectory is generated with velocities
        plotSim(simX, ref_traj, Nsim, save=True)
        plotPos(t,simX, ref_traj, Nsim, save=True)
        plotVel_with_vy_vz_references(t,simX, ref_traj, Nsim, save=True)
        plotSimU(t,simU, Nsim, save=True)

if ref_point and not import_trajectory: # For single reference points
    t = np.arange(0,T,Ts)
    plotSim_ref_point(simX, save = True)
    plotPos_ref_point(t,simX, save=True)
    plotVel(t,simX, Nsim, save=True)
    plotSimU(t,simU, Nsim, save=True)

if not ref_point and import_trajectory: # For imported trajectories with velocities and inputs
    
    # root mean squared error on each axis
    rmse_y, rmse_z = rmseX(simX, ref_traj[0:Nsim,:])

    # print the RMSE on each axis
    print("RMSE on y: {}".format(rmse_y))
    print("RMSE on z: {}".format(rmse_z))

    t = np.arange(0,T,Ts)
    plotSim(simX, ref_traj, Nsim, save=True)
    plotPos_with_ref(t,simX, ref_traj, Nsim, save=True)
    plotVel_with_ref(t,simX, ref_traj, Nsim, save=True)
    plotSimU_with_ref(t,simU, ref_U, Nsim, save=True)

plt.show()