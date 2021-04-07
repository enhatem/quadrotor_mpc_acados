from functools import update_wrapper
from acados_settings import acados_settings
import time, os
import numpy as np
import matplotlib.pyplot as plt
from plotFnc import *


Tf = 1 # prediction horizon
N = 10 # number of discretization steps 
T = 20.00 # simulation time[s]

# load model
model, acados_solver = acados_settings(Tf, N)

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)

# initialize data structs
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
tcomp_sum = 0
tcomp_max = 0

# simulate 
for i in range(Nsim):

    
    if i < 20: # stay at current position for 1 second
        for j in range(N):
            yref = np.array([0.3, 0, 9.81])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([0.3,0])
        acados_solver.set(N,"yref", yref_N)
    elif i >= 20 and i < 80:
        for j in range(N): # 
            yref = np.array([1, 0, 9.81])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([1,0])
        acados_solver.set(N,"yref", yref_N)
    elif i >= 80 and i < 140:
        for j in range(N): # 
            yref = np.array([0.5, 0, 9.81])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([0.5,0])
        acados_solver.set(N,"yref", yref_N)
    else:
        for j in range(N): # 
            yref = np.array([2.0, 0, 9.81])
            acados_solver.set(j, "yref", yref)
        yref_N = np.array([2.0,0])
        acados_solver.set(N,"yref", yref_N)


    # solve ocp for a fixed reference
    t = time.time()

    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))
    
    elapsed = time.time() - t

    # manage timings 
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed
    
    # get solution 
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")
    for j in range(nx):
        simX[i, j] = x0[j]
    for j in range(nu):
        simU[i, j] = u0[j]
    
    # update initial condition
    x0 = acados_solver.get(1,"x")
    acados_solver.set(0,"lbx", x0)
    acados_solver.set(0,"ubx", x0)

# Print some stats
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))

# Plot Results
t = np.linspace(0.0, Nsim * Tf / N, Nsim)
plotRes(simX, simU, t, save=True)
plt.show()