import numpy as np
import pyquaternion
import casadi as cs
import pandas as pd

from sklearn.metrics import mean_squared_error




def quaternion_to_euler(q):
    q = pyquaternion.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return np.array([qw, qx, qy, qz])


def unit_quat(q):
    """
    Normalizes a quaternion to be unit modulus.
    :param q: 4-dimensional numpy array or CasADi object
    :return: the unit quaternion in the same data format as the original one
    """

    if isinstance(q, np.ndarray):
        # if (q == np.zeros(4)).all():
        #     q = np.array([1, 0, 0, 0])
        q_norm = np.sqrt(np.sum(q ** 2))
    else:
        q_norm = cs.sqrt(cs.sumsqr(q))
    return 1 / q_norm * q


def R2D(rad):
    return rad*180 / np.pi

def add_measurement_noise(xcurrent):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)

    np.random.seed(12)

    y       = xcurrent[0]
    z       = xcurrent[1]
    phi     = xcurrent[2]
    vy      = xcurrent[3]
    vz      = xcurrent[4]
    phidot = xcurrent[5]

    # mean of the noise
    mean = 0

    # std of each state for a noiseless simulation
    std_y      = 0.01
    std_z      = 0.01
    std_phi    = 0.01
    std_vy     = 0.01
    std_vz     = 0.01
    std_phidot = 0.01
    
    # create the noisy states
    y_noisy      = y + np.random.normal(mean, std_y)
    z_noisy      = z + np.random.normal(mean, std_z)
    phi_noisy    = phi + np.random.normal(mean, std_phi)
    vy_noisy     = vy + np.random.normal(mean, std_vy)
    vz_noisy     = vz + np.random.normal(mean, std_vz)
    phidot_noisy = phidot + np.random.normal(mean, std_phidot)

    # create new noisy measurement vector
    xcurrent_noisy = np.array([y_noisy, z_noisy, phi_noisy, vy_noisy, vz_noisy, phidot_noisy])

    return xcurrent_noisy

def rmseX(simX, refX):
    rmse_y = mean_squared_error(refX[:,0], simX[1:,0], squared=False)
    rmse_z = mean_squared_error(refX[:,1], simX[1:,1], squared=False)

    return rmse_y, rmse_z

def saveData(simX, simU):
    
    # create the data frames
    datasetX = pd.DataFrame({'y': simX[:,0], 'z': simX[:,1], 'phi': simX[:,2], 'vy': simX[:,3], 'vz': simX[:,4], 'phi_dot': simX[:,5]})
    datasetU = pd.DataFrame({'Thrust': simU[:,0], 'Torque': simU[:,1]})

    # save data frames as .csv files
    datasetX.to_csv('saved_data/measX.csv')
    datasetU.to_csv('saved_data/simU.csv')

def getDynamics(m: float,
                Ixx: float,
                mean: np.array,   
                U: np.array):

    # constants
    g = 9.81 # m/s^2

    # state: X = [y, z, phi, vy, vz, phi_dot]
    
    # dynamic equations 
    # y_dot    = mean[3]                                  # y_dot   = vy
    # z_dot    = mean[4]                                  # z_dot   = vz
    #phi_dot  = mean[5]                                  # phi_dot = phi_dot
    vy_dot   = - ( U[0] / m ) * np.sin(mean[2])         # vy_dot = - u1/m * sin(phi)
    vz_dot   = - g + ( U[0] / m ) * np.cos(mean[2])     # vz_dot = -g + u1/m * cos(phi)
    phi_ddot = U[1] / Ixx                               # phi_ddot = u2 / Ixx

    # return np.array([y_dot, z_dot, phi_dot, vy_dot, vz_dot, phi_ddot])
    return np.array([vy_dot, vz_dot, phi_ddot])

def integrateArray(     f: np.array,
                        dt: float):
    delta_1 = f[0] * dt # delta_vy or delta_y
    delta_2 = f[1] * dt # delta_vz or delta_z
    delta_3 = f[2] * dt # delta_phiDot or delta_phi

    return np.array([delta_1, delta_2, delta_3])


def updateState(    x0: np.array,
                    delta_X: np.array, 
                    delta_vel: np.array) -> np.array:
    y       = x0[0] + delta_X[0]
    z       = x0[1] + delta_X[1]
    phi     = x0[2] + delta_X[2]
    vy      = x0[3] + delta_vel[0]
    vz      = x0[4] + delta_vel[1]
    phiDot  = x0[5] + delta_vel[2]

    return np.array([y, z, phi, vy, vz, phiDot])