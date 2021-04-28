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

def trajectory_generator2D( T_final, # simulation time 
                            Tf, # control horizon
                            Nsim, # number of time steps in the simulation 
                            N, # number of time step in a single time horizon
                            radius, # radius of the circle
                            show_traj=False):
    '''
    Generates a circular trajectory given a final time and a sampling time 
    '''
    phi = np.linspace(0,4*np.pi,Nsim+N)
    c_x, c_y = [4,5] # center coordinates 
    ## circular trajectory
    t  = np.linspace(0,T_final + Tf,Nsim+N)
    y  = radius * np.cos(phi) + c_x
    z  = radius * np.sin(phi) + c_y

    # y_end = y[-1] * np.ones_like(np.ndarray((N, 1)))
    # z_end = z[-1] * np.ones_like(np.ndarray((N, 1)))

    # y = np.append(y,y_end)
    # z = np.append(z,z_end)

    if show_traj == True:
        fig, ax = plt.subplots()
        plt.title('Reference trajectory')
        ax.plot(y, z)
        ax.set_xlabel("y[m]")
        ax.set_ylabel("z[m]")
        plt.show()

    return t,y,z


# trajectory generation with velocities
def trajectory_generotaor2D_with_vel(   model: object,
                                        radius: float, 
                                        freq: float, 
                                        T_final: float, 
                                        Tf: float,
                                        dt: float):

    t = np.arange(0,T_final+Tf,dt)

    c_y, c_z = [4,5] # center coordinates 

    y = radius * np.cos(freq * t) + c_y
    z = radius * np.sin(freq * t) + c_z

    vy  = - radius * freq * np.sin(freq * t)
    vz  = + radius * freq * np.cos(freq * t)

    v = np.sqrt(vy**2 + vz**2)

    # maximum velocity in the trajectory
    v_max = np.max(v) 
    
    if v_max > model.v_max:
        print('The desired trajectory contains velocities that the drone cannot handle.')
        return -1
    else:
        return t, y, z, vy, vz

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