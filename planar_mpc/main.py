from acados_settings import acados_settings
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from plotFnc import *
from utils import *
from trajectory import *


# mpc and simulation parameters
Tf = 1       # prediction horizon
N = 100      # number of discretization steps
T = 20.00    # simulation time[s]
Ts = Tf / N  # sampling time[s]

# constants
g = 9.81     # m/s^2

# noise bool
noisy_input = False

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(Ts, Tf, N)


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

# creating a reference trajectory
traj = 0 # traj = 0: circular trajectory, traj = 1: hellical trajectory
show_ref_traj = False
N_steps, x, y, z = trajectory_generator(T, Nsim, traj, show_ref_traj)
ref_traj = np.stack((x, y, z), 1)

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent
predX[0, :] = xcurrent

# create boolean to check if reference point is reached
ref_reached = False

# closed loop
for i in range(Nsim):

    '''
    # updating references
    for j in range(N):
        yref = np.array([x[i], y[i], z[i], 1.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, model.params.m * g, 0.0, 0.0, 0.0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([x[i], y[i], z[i], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    acados_solver.set(N, "yref", yref_N)
    '''

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
    xcurrent_pred = acados_solver.get(1, "x")
    u0 = acados_solver.get(0, "u")
    
    # add noise to inputs
    # if noisy_input == True:
    #     u0 = add_input_noise(u0, model)

    # storing results from acados solver
    predX[i+1, :] = xcurrent_pred
    simU[i, :] = u0

    # simulate the system
    acados_integrator.set("x", xcurrent)
    acados_integrator.set("u", u0)
    status = acados_integrator.solve()
    if status != 0:
        raise Exception(
            'acados integrator returned status {}. Exiting.'.format(status))
    
    
    # update state
    xcurrent = acados_integrator.get("x")
    simX[i+1, :] = xcurrent

    # check if y and z were reached
    if ref_reached == False:
        if (xcurrent[0] >= 1.99999 and xcurrent[0]<=2.00001) and (xcurrent[1] >= 3.99999 and xcurrent[1] <=4.00001):
            print(f'Reference reached after {i} iterations')
            print(f'Reference reached at simulation time of {i*Ts}s')
            print(f'phi={R2D(xcurrent[2])}deg')
            ref_reached = True

'''
# root mean squared error on each axis
rmse_x, rmse_y, rmse_z = rmseX(simX, ref_traj)
'''

# print the computation times
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))

'''
# print the RMSE on each axis
print("RMSE on x: {}".format(rmse_x))
print("RMSE on y: {}".format(rmse_y))
print("RMSE on z: {}".format(rmse_z))
'''


# Plot Results
t = np.linspace(0, T, Nsim)
plotSim(simX,save=True)
plotPos(t,simX,save=True)
plotVel(t,simX,save=True)
plt.show()


'''
plotSim_pos(t, simX, ref_traj, save=True)
plotSim_Angles(t, simX, simX_euler, save=True)
plotSim_vel(t, simX, save=True)
plotThrustInput(t, simU, save=True)
plotAngularRatesInputs(t, simU, save=True)
plotSim3D(simX, ref_traj, save=True)
plt.show()
'''