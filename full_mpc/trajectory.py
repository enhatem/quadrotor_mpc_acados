import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from numpy.core.numeric import ones_like

def trajectory_generator(T_final, N, traj=0, show_traj=False):
    '''
    Generates a circular trajectory given a final time and a sampling time 
    '''
    r = 1 # radius
    th = np.linspace(0,2*np.pi,N+100)
    c_x, c_y = [0,0] # center coordinates 
    ## circular trajectory
    if traj ==0: 
        t = np.linspace(0,T_final,N+100)
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


def trajectory_generator3D( x0: np.array,       # initial position of the quadrotor
                            N_hover: int,       # number of time steps in the hovering phase
                            N_traj: int,        # number of time steps in the simulation 
                            N: int,             # number of time step in a single time horizon (used to add an additional horizon in order for the closed loop simulation to work)
                            radius: float,      # radius of the circular trajectory
                            show_traj=False):   # boolean to show trajectory before the simulation
    '''
    Generates a 3D circular trajectory with a hovering time of T_hover at the start, 
    given a trajectory time and a sampling time.
    '''

    # hovering trajectory
    x_hover = np.ones(N_hover) * x0[0]
    y_hover = np.ones(N_hover) * x0[1]
    z_hover = np.ones(N_hover) * x0[2]


    # circular trajectory parameters
    phi = np.linspace(0,4*np.pi, N_traj+N)
    c_x, c_y = [0,0] # center coordinates 

    ## circular trajectory
    x_circle  = radius * np.cos(phi) + c_x
    y_circle  = radius * np.sin(phi) + c_y
    z_circle  = np.ones_like(x_circle)

    # appending the hovering and the circular trajectories
    x = np.append(x_hover, x_circle)
    y = np.append(y_hover, y_circle)
    z = np.append(z_hover, z_circle)
        
    if show_traj == True:
        fig, ax = plt.subplots()
        plt.title('Reference trajectory')
        ax = plt.axes(projection = "3d")
        ax.plot3D(x, y, z)
        ax.set_xlabel("x[m]")
        ax.set_ylabel("y[m]")
        ax.set_zlabel("z[m]")
        plt.show()

    return x,y,z


def trajectory_generotaor3D_with_vel(   x0: np.array,   # initial potision of the quadrotor
                                        N_hover: int,   # number of time steps in the hovering phase
                                        model: object,  # model of the drone (used to check if the maximum )
                                        radius: float,  # radius of the circular trajectory
                                        freq: float,    # used to control the speed of the trajectory
                                        T_traj: float,  # final time of the tre
                                        Tf: float,      # control horizon (used to add an additional horizon in order for the closed loop simulation to work)
                                        dt: float):

    # hovering trajectory
    x_hover     = np.ones(N_hover) * x0[0]
    y_hover     = np.ones(N_hover) * x0[1]
    z_hover     = np.ones(N_hover) * x0[2]
    vx_hover    = np.zeros(N_hover)
    vy_hover    = np.zeros(N_hover)
    vz_hover    = np.zeros(N_hover)
    
    t = np.arange(0,T_traj+Tf,dt)

    c_y, c_z = [0,0] # center coordinates 

    x_circle = radius * np.cos(freq * t) + c_y
    y_circle = radius * np.sin(freq * t) + c_z
    z_circle = np.ones_like(x_circle)

    vx_circle  = - radius * freq * np.sin(freq * t)
    vy_circle  = + radius * freq * np.cos(freq * t)
    vz_circle  = np.zeros_like(vx_circle)

    # appending the hovering and the circular trajectories
    x  = np.append(x_hover, x_circle)
    y  = np.append(y_hover, y_circle)
    z  = np.append(z_hover, z_circle)
    vx = np.append(vx_hover, vx_circle)
    vy = np.append(vy_hover, vy_circle)
    vz = np.append(vz_hover, vz_circle)


    v = np.sqrt(vy**2 + vz**2)

    # maximum velocity in the trajectory
    v_max = np.max(v) 
    
    if v_max > model.v_max:
        sys.exit("The desired trajectory contains velocities that the drone cannot handle.")
    else:
        return x, y, z, vx, vy, vz