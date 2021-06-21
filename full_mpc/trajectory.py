import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from numpy.core.numeric import ones_like
from utils import quaternion_to_euler, euler_to_quaternion, unit_quat

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

def readTrajectory(T_hover, N):
        
    # import csv file of measX and simU (noisy measurement)
    ref_traj = pd.read_csv('used_data/matlab/polynomial_5/fmincon4_solved/measX.csv')
    ref_U = pd.read_csv('used_data/matlab/polynomial_5/fmincon4_solved/simU.csv')
    
    # ref_traj = pd.read_csv('used_data/matlab/fmincon5/measX.csv')
    # ref_U = pd.read_csv('used_data/matlab/fmincon5/simU.csv')

    # create references to add for the hovering time
    ref_traj_x0 = ref_traj.iloc[[0]*N*T_hover]
    ref_u0 = ref_U.iloc[[0]*N*T_hover]

    # insert hovering references and inputs into their respective dataframes
    ref_traj = pd.concat([pd.DataFrame(ref_traj_x0), ref_traj], ignore_index=True)
    ref_U = pd.concat([pd.DataFrame(ref_u0), ref_U], ignore_index=True)
    
    # append last reference point 3*N times in order for the MPC controller to work at the last iteration (multiplication by 3 is not necessary for the simulation to work but will improve the results than just multilplying by 1)
    ref_traj = ref_traj.append( ref_traj.iloc[[-1]*N*3] )
    ref_U = ref_U.append( ref_U.iloc[[-1]*N*3] )

    # convert data frames to numpy arrays
    ref_traj = ref_traj[['y', 'z', 'phi', 'vy', 'vz', 'phi_dot']].to_numpy()
    ref_U = ref_U[['Thrust', 'Torque']].to_numpy()

    Thrust_ref = ref_U[:,0] * 2 # multiplied by 2 since the trajectory was based on a planar drone (mass/2)
    Torque_ref = ref_U[:,1] * 2 # not used but multiplied by 2 for consistency

    ref_U = np.array([Thrust_ref, Torque_ref]).T

    # extract each element of the trajectory
    x_ref     = np.zeros_like(ref_traj[:,0])
    y_ref     = ref_traj[:,0]
    z_ref     = ref_traj[:,1]
    phi_ref   = ref_traj[:,2]
    theta_ref = np.zeros_like(phi_ref)
    psi_ref   = np.zeros_like(phi_ref)
    vx_ref    = np.zeros_like(ref_traj[:,0])
    vy_ref    = ref_traj[:,3]
    vz_ref    = ref_traj[:,4]

    quat_ref = euler_to_quaternion(phi_ref,theta_ref,psi_ref).T

    # Number of rows in the trajectory
    rows = phi_ref.shape[0]

    # Ensure the unit quaternion at each iteration
    for i in range(rows):
        quat_ref[i] = unit_quat(quat_ref[i])

    # Extract the elements of the quaternion
    qw_ref = quat_ref[:,0]
    qx_ref = quat_ref[:,1]
    qy_ref = quat_ref[:,2]
    qz_ref = quat_ref[:,3]

    ref_traj = np.array([x_ref, y_ref, z_ref, qw_ref, qx_ref, qy_ref, qz_ref, vx_ref, vy_ref, vz_ref]).T

    # computing simulation time
    T = ((len(ref_traj) ) / N)

    return T, ref_traj , ref_U