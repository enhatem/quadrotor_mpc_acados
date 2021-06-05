import numpy as np
import pyquaternion
import casadi as cs
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

def add_measurement_noise(xcurrent):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)

    qw = xcurrent[3]
    qx = xcurrent[4]
    qy = xcurrent[5]
    qz = xcurrent[6]

    quat = np.array([qw,qx,qy,qz])
    euler_angles = quaternion_to_euler(quat)

    x     = xcurrent[0]
    y     = xcurrent[1]
    z     = xcurrent[2]
    roll  = euler_angles[0]
    pitch = euler_angles[1]
    yaw   = euler_angles[2]
    vx    = xcurrent[7]
    vy    = xcurrent[8]
    vz    = xcurrent[9]

    # mean of the noise
    mean = 0

    # scale of noise of each state
    std_x     = 0.01
    std_y     = 0.01
    std_z     = 0.01
    std_roll  = (np.pi / 180) / 2 
    std_pitch = (np.pi / 180) / 2
    std_yaw   = (np.pi / 180) / 2
    std_vx    = 0.001
    std_vy    = 0.001
    std_vz    = 0.001

    # create the noisy states
    x_noisy     =  x + np.random.normal(mean, std_x)
    y_noisy     =  y + np.random.normal(mean, std_y)
    z_noisy     =  z + np.random.normal(mean, std_z)
    roll_noisy  =  roll + np.random.normal(mean, std_roll)
    pitch_noisy =  pitch + np.random.normal(mean, std_pitch)
    yaw_noisy   =  yaw + np.random.normal(mean, std_yaw)
    vx_noisy    =  vx + np.random.normal(mean, std_vx)
    vy_noisy    =  vy + np.random.normal(mean, std_vy)
    vz_noisy    =  vz + np.random.normal(mean, std_vz)

    q_noisy = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)
    q_noisy = unit_quat(q_noisy)
    qw_noisy = q_noisy[0]
    qx_noisy = q_noisy[1]
    qy_noisy = q_noisy[2]
    qz_noisy = q_noisy[3]

    # create new noisy measurement vector
    xcurrent_noisy = np.array([x_noisy, y_noisy, z_noisy, qw_noisy, qx_noisy, qy_noisy, qz_noisy, vx_noisy, vy_noisy, vz_noisy])

    return xcurrent_noisy


def ensure_unit_quat(xcurrent):
    # ensure that the quaternion in the current state is a unit vector
    x  = xcurrent[0]
    y  = xcurrent[1]
    z  = xcurrent[2]
    qw = xcurrent[3]
    qx = xcurrent[4]
    qy = xcurrent[5]
    qz = xcurrent[6]
    vx = xcurrent[7]
    vy = xcurrent[8]
    vz = xcurrent[9]

    q = np.array([qw, qx, qy, qz])
    q = unit_quat(q)
    
    # extracting the elements from q
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    xcurrent = np.array([x, y, z, qw, qx, qy, qz, vx, vy, vz])

    return xcurrent


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

def add_input_noise(u0,model):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)
    T = np.array([u0[0]])
    w = u0[1:]

    mean = 0
    std_T = 0.01
    std_w = np.std(w)
    
    # std_q = np.std(q)
    T_noisy = T + np.random.normal(mean, std_T)
    T_noisy = max(min(T_noisy ,model.throttle_max), model.throttle_min)

    '''
    roll_noisy = roll + np.random.normal(mean, std_Angles)
    pitch_noisy = pitch + np.random.normal(mean, std_Angles)
    yaw_noisy = yaw + np.random.normal(mean, std_Angles)

    q_noisy = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)

    # ensure that q_noisy is of unit modulus 
    q_noisy = unit_quat(q_noisy)
    '''
    w_noisy = np.zeros_like(w)
    for i, ui in enumerate(w):
        w_noisy[i] = ui + np.random.normal(mean, std_w)
    
    # create new noisy input vector
    u_noisy = np.append(T_noisy,w_noisy)

    return u_noisy

def rmseX(simX, refX):
    rmse_x = mean_squared_error(refX[:,0], simX[1:,0], squared=False)
    rmse_y = mean_squared_error(refX[:,1], simX[1:,1], squared=False)
    rmse_z = mean_squared_error(refX[:,2], simX[1:,2], squared=False)

    return rmse_x, rmse_y, rmse_z