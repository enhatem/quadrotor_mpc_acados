import time
import numpy as np
import matplotlib.pyplot as plt

from acados_settings import acados_settings
from plotFnc import *
from utils import *
from trajectory import *


# mpc and simulation parameters
Tf = 1       # prediction horizon
N = 25       # number of discretization steps
T = 20.00    # simulation time[s]
Ts = Tf / N  # sampling time[s]

# constants
g = 9.81     # m/s^2

# noise bool
noisy_measurement = False

# use acados integrator:
use_acados_integrator = False

# bool to save measurements and inputs as .csv files
save_data = True

# load model and acados_solver
model, acados_solver, acados_integrator = acados_settings(Ts, Tf, N)

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

# creating a reference trajectory
traj = 0 # traj = 0: circular trajectory, traj = 1: hellical trajectory
show_ref_traj = False
t, y, z = trajectory_generator2D(T, Nsim, traj, show_ref_traj)

y_end = y[-1] * np.ones_like(np.ndarray((N, 1)))
z_end = z[-1] * np.ones_like(np.ndarray((N, 1)))
# vy_end = vy[-1] * np.ones_like(np.ndarray((N, 1)))
# vz_end = vz[-1] * np.ones_like(np.ndarray((N, 1)))

y = np.append(y,y_end)
z = np.append(z,z_end)
# vy = np.append(vy,vy_end)
# vz = np.append(vz,vz_end)

ref_traj = np.stack((y, z), 1)

# set initial condition for acados integrator
xcurrent = model.x0.reshape((nx,))
simX[0, :] = xcurrent
# predX[0, :] = xcurrent

# create boolean to check if reference point is reached
ref_reached = False
ref_xy_reached = False
ref_phi_reached = False

# closed loop
for i in range(Nsim):

#     if i == (Nsim-N+1):
#         break
    # updating references
    for j in range(N):
        yref = np.array([y[i+j], z[i+j], 0.0, 0.0, 0.0, 0.0, model.params.m * g, 0.0])
        acados_solver.set(j, "yref", yref)
    yref_N = np.array([y[i+N], z[i+N], 0.0, 0.0, 0.0, 0.0])
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
    # xcurrent_pred = acados_solver.get(1, "x")
    u0 = acados_solver.get(0, "u")


    # storing results from acados solver
    # predX[i+1, :] = xcurrent_pred
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



# Plot Results
# N_step = np.linspace(0, 19, Nsim-N)
#simX = simX[0:Nsim-N+1,:]
#simU = simU[0:Nsim-N,:]
#ref_traj = ref_traj[0:Nsim-N,:]

ref_traj = ref_traj[0:Nsim,:]

# root mean squared error on each axis
rmse_y, rmse_z = rmseX(simX, ref_traj)

# print the computation times
print("Total computation time: {}".format(tot_comp_sum))
print("Average computation time: {}".format(tot_comp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))


# print the RMSE on each axis
# print("RMSE on x: {}".format(rmse_x))
print("RMSE on y: {}".format(rmse_y))
print("RMSE on z: {}".format(rmse_z))


plotSim(simX, ref_traj, save=True)
plotPos(t,simX, ref_traj, save=True)
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