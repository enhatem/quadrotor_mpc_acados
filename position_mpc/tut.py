from trajectory import trajectory_generator
import numpy as np
import matplotlib.pyplot as plt

T_final = 10
N = 100
traj = 1
plot = True

t,x,y,z = trajectory_generator(T_final, N, traj, plot)
plt.show()