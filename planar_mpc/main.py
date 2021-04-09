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
noisy_measurement = False

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
N_steps, y, z = trajectory_generator2D(T, Nsim, traj, show_ref_traj)
ref_traj = np.stack((y, z), 1)

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent
predX[0, :] = xcurrent

# create boolean to check if reference point is reached
ref_reached = False
ref_xy_reached = False
ref_phi_reached = False

# closed loop
for i in range(Nsim):

    
    # updating references
    for j in range(N):
        yref = np.array([y[i], z[i], 0.0, 0.0, 0.0, 0.0, model.params.m * g, 0.0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([y[i], z[i], 0.0, 0.0, 0.0, 0.0])
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
    xcurrent_pred = acados_solver.get(1, "x")
    u0 = acados_solver.get(0, "u")


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
    
    
    # get state
    xcurrent = acados_integrator.get("x")

    # add measurement noise
    if noisy_measurement == True:
        xcurrent = add_measurement_noise(xcurrent)

    # update state
    simX[i+1, :] = xcurrent
    
    '''
    # check if y and z were reached with hovering condition (phi between -0.01deg and 0.01deg)
    if ref_reached == False:
        if (xcurrent[0] >= 1.99 and xcurrent[0]<=2.01) and (xcurrent[1] >= 3.99 and xcurrent[1] <=4.01):
            if ref_xy_reached == False:
                print(f'xy reference reached after {i} iterations')
                print(f'xy reference reached at simulation time of {i*Ts}s')
                # ref_xy_reached = True
            if ref_phi_reached == False:
                if (R2D(xcurrent[2]>=-0.01 and R2D(xcurrent[2]<=0.01))):
                    print(f'phi reference reached after {i} iterations')
                    print(f'phi reference reached at simulation time of {i*Ts}s')
                    #ref_phi_reached = True
                    #ref_reached = True
    '''

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
plotSim(simX, ref_traj, save=True)
plotPos(t,simX,save=True)
plotVel(t,simX,save=True)
plotSimU(t,simU,save=True)
plt.show()


'''
## Live plot
plt.style.use('fivethirtyeight')

index = count()

def animate(i):
    y_vals = simX[:,0]
    z_vals = simX[:,1]
    
    plt.cla() # to clear the axis
    plt.plot(y_vals, z_vals)

    plt.legend(loc='upper left')
    plt.tight_layout()

ani = FuncAnimation(plt.gcf(), animate, interval=5000)

plt.tight_layout()
plt.show()
'''