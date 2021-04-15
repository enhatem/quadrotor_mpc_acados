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