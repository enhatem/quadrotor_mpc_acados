from acados_settings import acados_settings
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from plotFnc import *
from utils import *
from trajectory import *


# mpc and simulation parameters
Tf = 1        # prediction horizon
N = 50      # number of discretization steps
Ts = Tf / N   # sampling time[s]

T_hover = 2     # hovering time[s]
T_traj = 10.00  # trajectory time[s]

T = T_hover + T_traj  # total simulation time

# constants
g = 9.81     # m/s^2

# measurement noise bool
noisy_measurement = False

# input noise bool
noisy_input = False

# extended kalman filter bool
# extended_kalman_filter = False

# generate circulare trajectory with velocties
traj_with_vel = False

# use a single reference point
ref_point = False

# import trajectory with positions and velocities and inputs
import_trajectory = True

# bool to save measurements and inputs as .csv files
save_data = True

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(Ts, Tf, N)

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
N_hover = int(T_hover * N / Tf)
N_traj = int(T_traj * N / Tf)
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))
tot_comp_sum = 0
tcomp_max = 0

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent

## creating or extracting trajectory

# circular trajectory
if ref_point == False and import_trajectory == False:
    # creating a reference trajectory
    show_ref_traj = False
    radius = 1  # m
    freq = 14 * np.pi/10  # frequency

    # without velocity
    if traj_with_vel == False:
        x, y, z = trajectory_generator3D(
            xcurrent, N_hover, N_traj, N, radius, show_ref_traj)
        ref_traj = np.stack((x, y, z), 1)
    else:
        # with velocity
        x, y, z, vx, vy, vz = trajectory_generotaor3D_with_vel(
            xcurrent, N_hover, model, radius, freq, T_traj, Tf, Ts)
        ref_traj = np.stack((x, y, z, vx, vy, vz), 1)

# reference point
elif ref_point == True and import_trajectory == False:
    X0 = xcurrent
    x_ref_point = 0
    y_ref_point = 1.2
    z_ref_point = 1.0
    X_ref = np.array([x_ref_point, y_ref_point, z_ref_point])

# imported trajectory
elif ref_point == False and import_trajectory == True:
    T, ref_traj, ref_U, w_ref = readTrajectory(T_hover, N, Ts)
    Nsim = int((T-Tf) * N / Tf)
    predX = np.ndarray((Nsim+1, nx))
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim,   nu))
    simX[0, :] = xcurrent

'''
elif ref_point == False and import_trajectory == True:
    T, ref_traj, ref_U = readTrajectory(T_hover, N)
    Nsim = int((T-Tf) * N / Tf)
    predX = np.ndarray((Nsim+1, nx))
    simX  = np.ndarray((Nsim+1, nx))
    simU  = np.ndarray((Nsim,   nu))
    simX[0, :] = xcurrent
'''


# N_steps, x, y, z = trajectory_generator(T, Nsim, traj, show_ref_traj)
# ref_traj = np.stack((x, y, z), 1)


# closed loop
for i in range(Nsim):

    # updating references
    if ref_point == False and import_trajectory == False:
        if traj_with_vel == False:
            for j in range(N):
                yref = np.array([x[i+j], y[i+j], z[i+j], 1.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, model.params.m * g, 0.0, 0.0, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([x[i+N], y[i+N], z[i+N], 1.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            acados_solver.set(N, "yref", yref_N)
        else:
            for j in range(N):
                yref = np.array([x[i+j], y[i+j], z[i+j], 1.0, 0.0, 0.0, 0.0,
                                 vx[i+j], vy[i+j], vz[i+j], model.params.m * g, 0.0, 0.0, 0.0])
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([x[i+N], y[i+N], z[i+N], 1.0,
                               0.0, 0.0, 0.0, vx[i+N], vy[i+N], vz[i+N]])
            acados_solver.set(N, "yref", yref_N)

    elif ref_point == True and import_trajectory == False:
        for j in range(N):
            if i < N_hover:
                yref = np.array([X0[0], X0[1], X0[2], 1.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, model.params.m * g, 0.0, 0.0, 0.0])
                acados_solver.set(j, "yref", yref)
            else:
                yref = np.array([x_ref_point, y_ref_point, z_ref_point, 1.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, model.params.m * g, 0.0, 0.0, 0.0])
                acados_solver.set(j, "yref", yref)
        if i < N_hover:
            yref_N = np.array(
                [X0[0], X0[1], X0[2], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            acados_solver.set(N, "yref", yref_N)
        else:
            yref_N = np.array(
                [x_ref_point, y_ref_point, z_ref_point, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            acados_solver.set(N, "yref", yref_N)

    elif ref_point == False and import_trajectory == True:
        # if i == Nsim-5:
        #     print(f'i={i}')

        for j in range(N):
            x = ref_traj[i+j, 0]
            y = ref_traj[i+j, 1]
            z = ref_traj[i+j, 2]
            qw = ref_traj[i+j, 3]
            qx = ref_traj[i+j, 4]
            qy = ref_traj[i+j, 5]
            qz = ref_traj[i+j, 6]
            vx = ref_traj[i+j, 7]
            vy = ref_traj[i+j, 8]
            vz = ref_traj[i+j, 9]

            T_ref = ref_U[i+j, 0] * 2  # Thrust
            wx_ref = w_ref[i+j,0]  # Angular rate around x
            wy_ref = w_ref[i+j,1]  # Angular rate around y
            wz_ref = w_ref[i+j,2]  # Angular rate around z

            yref = np.array([x, y, z, qw, qx, qy, qz, vx, vy,
                             vz, T_ref, wx_ref, wy_ref, wz_ref])
            acados_solver.set(j, "yref", yref)

        x_e  = ref_traj[i+N, 0]
        y_e  = ref_traj[i+N, 1]
        z_e  = ref_traj[i+N, 2]
        qw_e = ref_traj[i+N, 3]
        qx_e = ref_traj[i+N, 4]
        qy_e = ref_traj[i+N, 5]
        qz_e = ref_traj[i+N, 6]
        vx_e = ref_traj[i+N, 7]
        vy_e = ref_traj[i+N, 8]
        vz_e = ref_traj[i+N, 9]

        yref_N = np.array([x_e, y_e, z_e, qw_e, qx_e,
                           qy_e, qz_e, vx_e, vy_e, vz_e])
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
    # x4 = acados_solver.get(4, "x") # used to compensate for delays

    # storing results from acados solver
    simU[i, :] = u0

    # add noise to measurement
    if noisy_measurement == True:
        xcurrent = add_measurement_noise(xcurrent)

    # simulate the system
    acados_integrator.set("x", xcurrent)
    acados_integrator.set("u", u0)
    status = acados_integrator.solve()
    if status != 0:
        raise Exception(
            'acados integrator returned status {}. Exiting.'.format(status))

    # get state
    xcurrent = acados_integrator.get("x")

    # make sure that the quaternion is unit
    # xcurrent = ensure_unit_quat(xcurrent)

    # store state
    simX[i+1, :] = xcurrent

# root mean squared error on each axis
# rmse_x, rmse_y, rmse_z = rmseX(simX, ref_traj)

# print the computation times
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))

simX_euler = np.zeros((simX.shape[0], 3))

for i in range(simX.shape[0]):
    simX_euler[i, :] = quaternion_to_euler(simX[i, 3:7])

simX_euler = R2D(simX_euler)
# print(simX_euler)

if import_trajectory == False:
    t = np.arange(0, T, Ts)
else:
    t = np.arange(0, T-Tf, Ts)


if ref_point == False and import_trajectory == False:
    # Plot Results
    if traj_with_vel == False:
        plotSim3D(simX, ref_traj, save=True)
        plotSim_pos(t, simX, ref_traj, Nsim, save=True)
        plotSim_Angles(t, simX, simX_euler, Nsim, save=True)
        plotSim_vel(t, simX, Nsim, save=True)
        plotThrustInput(t, simU, Nsim, save=True)
        plotAngularRatesInputs(t, simU, Nsim, save=True)
        plotErrors_no_vel(t, simX, ref_traj, Nsim, save=True)
    elif traj_with_vel == True:
        plotSim3D(simX, ref_traj, save=True)
        plotSim_pos(t, simX, ref_traj, Nsim, save=True)
        plotSim_Angles(t, simX, simX_euler, Nsim, save=True)
        plotSim_vel_with_ref(t, simX, ref_traj, Nsim, save=True)
        plotThrustInput(t, simU, Nsim, save=True)
        plotAngularRatesInputs(t, simU, Nsim, save=True)
        plotErrors_with_vel(t, simX, ref_traj, Nsim, save=True)

elif ref_point == True and import_trajectory == False:
    plotSim3D_ref_point(simX, X_ref, save=True)
    plotSim_pos_ref_point(t, simX, Nsim, save=True)
    plotSim_Angles(t, simX, simX_euler, Nsim, save=True)
    plotSim_vel(t, simX, Nsim, save=True)
    plotThrustInput(t, simU, Nsim, save=True)
    plotAngularRatesInputs(t, simU, Nsim, save=True)

elif ref_point == False and import_trajectory == True:
    plotSim3D(simX, ref_traj, save=True)
    plotSim_pos(t, simX, ref_traj, Nsim, save=True)
    plotSim_Angles(t, simX, simX_euler, Nsim, save=True)
    plotSim_vel_with_ref_for_imported_trajectory(
        t, simX, ref_traj, Nsim, save=True)
    plotThrustInput_with_ref(t, simU, ref_U, Nsim, save=True)
    plotAngularRatesInputs(t, simU, w_ref, Nsim, save=True)
    plotErrors_with_vel(t, simX, ref_traj, Nsim, save=True)
plt.show()
