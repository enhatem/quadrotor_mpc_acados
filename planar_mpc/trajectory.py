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
                            N_traj: int,          # number of time steps in the simulation 
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
    theta = np.linspace(0,4*np.pi, N_traj+N)
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

def readTrajectory(T_hover, N):
        
    # import csv file of measX and simU (noisy measurement)
    ref_traj = pd.read_csv('used_data/matlab/ga4/measX.csv')
    ref_U = pd.read_csv('used_data/matlab/ga4/simU.csv')
    
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

    # computing simulation time
    T = (len(ref_traj) ) / N

    return T, ref_traj, ref_U

'''
def loop_trajectory(quad, discretization_dt, radius, z, lin_acc, clockwise, yawing, v_max, map_name, plot):
    """
    Creates a circular trajectory on the x-y plane that increases speed by 1m/s at every revolution.

    :param quad: Quadrotor model
    :param discretization_dt: Sampling period of the trajectory.
    :param radius: radius of loop trajectory in meters
    :param z: z position of loop plane in meters
    :param lin_acc: linear acceleration of trajectory (and successive deceleration) in m/s^2
    :param clockwise: True if the rotation will be done clockwise.
    :param yawing: True if the quadrotor yaws along the trajectory. False for 0 yaw trajectory.
    :param v_max: Maximum speed at peak velocity. Revolutions needed will be calculated automatically.
    :param map_name: Name of map to load its limits
    :param plot: Whether to plot an analysis of the planned trajectory or not.
    :return: The full 13-DoF trajectory with time and input vectors
    """

    ramp_up_t = 2  # s

    # Calculate simulation time to achieve desired maximum velocity with specified acceleration
    t_total = 2 * v_max / lin_acc + 2 * ramp_up_t

    # Transform to angular acceleration
    alpha_acc = lin_acc / radius  # rad/s^2

    # Generate time and angular acceleration sequences
    # Ramp up sequence
    ramp_t_vec = np.arange(0, ramp_up_t, discretization_dt)
    ramp_up_alpha = alpha_acc * np.sin(np.pi / (2 * ramp_up_t) * ramp_t_vec) ** 2
    # Acceleration phase
    coasting_duration = (t_total - 4 * ramp_up_t) / 2
    coasting_t_vec = ramp_up_t + np.arange(0, coasting_duration, discretization_dt)
    coasting_alpha = np.ones_like(coasting_t_vec) * alpha_acc
    # Transition phase: decelerate
    transition_t_vec = np.arange(0, 2 * ramp_up_t, discretization_dt)
    transition_alpha = alpha_acc * np.cos(np.pi / (2 * ramp_up_t) * transition_t_vec)
    transition_t_vec += coasting_t_vec[-1] + discretization_dt
    # Deceleration phase
    down_coasting_t_vec = transition_t_vec[-1] + np.arange(0, coasting_duration, discretization_dt) + discretization_dt
    down_coasting_alpha = -np.ones_like(down_coasting_t_vec) * alpha_acc
    # Bring to rest phase
    ramp_up_t_vec = down_coasting_t_vec[-1] + np.arange(0, ramp_up_t, discretization_dt) + discretization_dt
    ramp_up_alpha_end = ramp_up_alpha - alpha_acc

    # Concatenate all sequences
    t_ref = np.concatenate((ramp_t_vec, coasting_t_vec, transition_t_vec, down_coasting_t_vec, ramp_up_t_vec))
    alpha_vec = np.concatenate((
        ramp_up_alpha, coasting_alpha, transition_alpha, down_coasting_alpha, ramp_up_alpha_end))

    # Calculate derivative of angular acceleration (alpha_vec)
    ramp_up_alpha_dt = alpha_acc * np.pi / (2 * ramp_up_t) * np.sin(np.pi / ramp_up_t * ramp_t_vec)
    coasting_alpha_dt = np.zeros_like(coasting_alpha)
    transition_alpha_dt = - alpha_acc * np.pi / (2 * ramp_up_t) * np.sin(np.pi / (2 * ramp_up_t) * transition_t_vec)
    alpha_dt = np.concatenate((
        ramp_up_alpha_dt, coasting_alpha_dt, transition_alpha_dt, coasting_alpha_dt, ramp_up_alpha_dt))

    if not clockwise:
        alpha_vec *= -1
        alpha_dt *= -1

    # Compute angular integrals
    w_vec = np.cumsum(alpha_vec) * discretization_dt
    angle_vec = np.cumsum(w_vec) * discretization_dt

    # Compute position, velocity, acceleration, jerk
    pos_traj_x = radius * np.sin(angle_vec)[np.newaxis, np.newaxis, :]
    pos_traj_y = radius * np.cos(angle_vec)[np.newaxis, np.newaxis, :]
    pos_traj_z = np.ones_like(pos_traj_x) * z

    vel_traj_x = (radius * w_vec * np.cos(angle_vec))[np.newaxis, np.newaxis, :]
    vel_traj_y = - (radius * w_vec * np.sin(angle_vec))[np.newaxis, np.newaxis, :]

    acc_traj_x = radius * (alpha_vec * np.cos(angle_vec) - w_vec ** 2 * np.sin(angle_vec))[np.newaxis, np.newaxis, :]
    acc_traj_y = - radius * (alpha_vec * np.sin(angle_vec) + w_vec ** 2 * np.cos(angle_vec))[np.newaxis, np.newaxis, :]

    jerk_traj_x = radius * (alpha_dt * np.cos(angle_vec) - alpha_vec * np.sin(angle_vec) * w_vec -
                            np.cos(angle_vec) * w_vec ** 3 - 2 * np.sin(angle_vec) * w_vec * alpha_vec)
    jerk_traj_y = - radius * (np.cos(angle_vec) * w_vec * alpha_vec + np.sin(angle_vec) * alpha_dt -
                              np.sin(angle_vec) * w_vec ** 3 + 2 * np.cos(angle_vec) * w_vec * alpha_vec)
    jerk_traj_x = jerk_traj_x[np.newaxis, np.newaxis, :]
    jerk_traj_y = jerk_traj_y[np.newaxis, np.newaxis, :]

    if yawing:
        yaw_traj = -angle_vec
    else:
        yaw_traj = np.zeros_like(angle_vec)

    traj = np.concatenate((
        np.concatenate((pos_traj_x, pos_traj_y, pos_traj_z), 1),
        np.concatenate((vel_traj_x, vel_traj_y, np.zeros_like(vel_traj_x)), 1),
        np.concatenate((acc_traj_x, acc_traj_y, np.zeros_like(acc_traj_x)), 1),
        np.concatenate((jerk_traj_x, jerk_traj_y, np.zeros_like(jerk_traj_x)), 1)), 0)

    yaw = np.concatenate((yaw_traj[np.newaxis, :], w_vec[np.newaxis, :]), 0)

    return minimum_snap_trajectory_generator(traj, yaw, t_ref, quad, map_limits, plot)
'''