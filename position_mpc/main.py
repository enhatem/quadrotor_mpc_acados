from functools import update_wrapper
from acados_settings import acados_settings
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from plotFnc import *
from utils import *
from trajectory import *


Tf = 1  # prediction horizon
N = 100  # number of discretization steps
T = 20.00  # simulation time[s]
Ts = Tf / N  # sampling time[s]

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(Ts, Tf, N)


# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
predX = np.ndarray((Nsim+1, nx))
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))
tot_comp_sum = 0
tcomp_max = 0

# creating a reference trajectory
traj = 0 # traj = 0: circular trajectory, traj = 1: spiral trajectory
show_ref_traj = False
N_steps, x, y, z = trajectory_generator(T, Nsim, traj, show_ref_traj)
ref_traj = np.stack((x,y,z),1)

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0,:] = xcurrent
predX[0,:] = xcurrent

# closed loop
for i in range(Nsim):

    for j in range(N):
        yref = np.array([x[i], y[i], z[i], 0.0, 0.0,
                         0.0, 9.81, 1.0, 0.0, 0.0, 0.0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([x[i], y[i], z[i], 0.0, 0.0, 0.0])
    acados_solver.set(N, "yref", yref_N)

    # solve ocp for a fixed reference
    acados_solver.set(0, "lbx", xcurrent)
    acados_solver.set(0, "ubx", xcurrent)
    comp_time = time.time()
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    elapsed = time.time() - comp_time

    # manage timings
    tot_comp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution from acados_solver
    xcurrent_pred = acados_solver.get(1, "x")
    u0 = acados_solver.get(0, "u")
    predX[i+1,:] = xcurrent_pred
    simU[i,:] = u0

    # simulate the system
    acados_integrator.set("x", xcurrent)
    acados_integrator.set("u", u0)
    # add disturbance to the system (will be done later)
    '''
    pertubation = sampleFromEllipsoid(np.zeros((nparam,)), W)
    acados_integrator.set("p", pertubation)
    '''
    status = acados_integrator.solve()
    if status != 0:
        raise Exception('acados integrator returned status {}. Exiting.'.format(status))
    # update state
    xcurrent = acados_integrator.get("x")
    simX[i+1,:] = xcurrent

# print the computation times
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))


simU_euler = np.zeros((simU.shape[0], 3))

for i in range(simU.shape[0]):
    simU_euler[i, :] = quaternion_to_euler(simU[i, 1:])

simU_euler = R2D(simU_euler)

# Plot Results

t = np.linspace(0,T, Nsim)
plotSim_pos(simX, predX, ref_traj)
plotSim_vel(simX, predX)
plotThrustInputs(t, simU)
plotAngleInputs(t,simU, simU_euler)
plotSim3D(simX, predX, ref_traj)

plt.show()
