import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from mpl_toolkits import mplot3d

def trajectory_generator(T_final, N, traj=0, show_traj=False):
    '''
    Generates a circular trajectory given a final time and a sampling time 
    '''
    r = 1 # radius
    th = np.linspace(0,6*np.pi,N)
    c_x, c_y = [0,0] # center coordinates 
    ## circular trajectory
    if traj ==0: 
        t = np.linspace(0,T_final,N)
        x = r * np.cos(th) + c_x
        y = r * np.sin(th) + c_y
        z = np.ones_like(th)

        if show_traj == True:
            plt.figure()
            ax = plt.axes(projection = "3d")
            plt.title('Reference trajectory')
            ax.plot3D(x, y, z)
            ax.set_xlabel("x[m]")
            ax.set_ylabel("y[m]")
            ax.set_zlabel("z[m]")
            plt.show()    

    ## hellical trajectory
    if traj ==1: 
        t = np.linspace(0,T_final,N)
        x = r * np.cos(th) + c_x
        y = r * np.sin(th) + c_y
        z = np.linspace(1,2,N)

        if show_traj == True:
            plt.figure()
            ax = plt.axes(projection = "3d")
            plt.title('Reference trajectory')
            ax.plot3D(x, y, z)
            plt.show()

    ## vertical trajectory
    if traj ==2: 
        t = np.linspace(0,T_final,N)
        x = np.ones_like(t)
        y = np.zeros_like(t)
        z = np.linspace(1,2,N)

        if show_traj == True:
            plt.figure()
            ax = plt.axes(projection = "3d")
            plt.title('Reference trajectory')
            ax.plot3D(x, y, z)
            plt.show()
    



    return t,x,y,z

def trajectory_generator2D( x0: np.array,       # initial position of the quadrotor
                            N_hover: int,       # number of time steps in the hovering phase
                            Nsim: int,          # number of time steps in the simulation 
                            N: int,             # number of time step in a single time horizon (used to add an additional horizon in order for the closed loop simulation to work)
                            radius: float,      # radius of the circular trajectory
                            show_traj=False):   # boolean to show trajectory before the simulation
    '''
    Generates a circular trajectory with a hovering time of T_hover at the start, 
    given a trajectory time and a sampling time.
    '''

    # hovering trajectory
    y_hover = np.ones(N_hover) * x0[0]
    z_hover = np.ones(N_hover) * x0[1]

    # circular trajectory parameters
    theta = np.linspace(0,4*np.pi,Nsim+N)
    c_x, c_y = [4,5] # center coordinates 

    ## circular trajectory
    y_circle  = radius * np.cos(theta) + c_x
    z_circle  = radius * np.sin(theta) + c_y

    # appending the hovering and the circular trajectories
    y = np.append(y_hover, y_circle)
    z = np.append(z_hover, z_circle)
        
    if show_traj == True:
        fig, ax = plt.subplots()
        plt.title('Reference trajectory')
        ax.plot(y, z)
        ax.set_xlabel("y[m]")
        ax.set_ylabel("z[m]")
        plt.show()

    return y,z


# trajectory generation with velocities
def trajectory_generotaor2D_with_vel(   x0: np.array,   # initial potision of the quadrotor
                                        N_hover: int,   # number of time steps in the hovering phase
                                        model: object,  # model of the drone (used to check if the maximum )
                                        radius: float,  # radius of the circular trajectory
                                        freq: float,    # used to control the speed of the trajectory
                                        T_traj: float,  # final time of the tre
                                        Tf: float,      # control horizon (used to add an additional horizon in order for the closed loop simulation to work)
                                        dt: float):

    # hovering trajectory
    y_hover     = np.ones(N_hover) * x0[0]
    z_hover     = np.ones(N_hover) * x0[1]
    vz_hover    = np.zeros(N_hover)
    vy_hover    = np.zeros(N_hover)
    
    t = np.arange(0,T_traj+Tf,dt)

    c_y, c_z = [4,5] # center coordinates 

    y_circle = radius * np.cos(freq * t) + c_y
    z_circle = radius * np.sin(freq * t) + c_z

    vy_circle  = - radius * freq * np.sin(freq * t)
    vz_circle  = + radius * freq * np.cos(freq * t)

    # appending the hovering and the circular trajectories
    y  = np.append(y_hover, y_circle)
    z  = np.append(z_hover, z_circle)
    vy = np.append(vy_hover, vy_circle)
    vz = np.append(vz_hover, vz_circle)


    v = np.sqrt(vy**2 + vz**2)

    # maximum velocity in the trajectory
    v_max = np.max(v) 
    
    if v_max > model.v_max:
        sys.exit("The desired trajectory contains velocities that the drone cannot handle.")
    else:
        return y, z, vy, vz

def readTrajectory(N):
        
    # import csv file of measX and simU (noisy measurement)
    ref_traj = pd.read_csv('used_data/measX.csv')
    ref_U = pd.read_csv('used_data/simU.csv')
    
    ref_traj = ref_traj.append( ref_traj.iloc[[-1]*N] )
    ref_U    = ref_U.append( ref_U.iloc[[-1]*N] )

    # convert data frames to numpy arrays
    ref_traj = ref_traj[['y', 'z', 'phi', 'vy', 'vz', 'phi_dot']].to_numpy()
    ref_U = ref_U[['Thrust', 'Torque']].to_numpy()

    return ref_traj, ref_U