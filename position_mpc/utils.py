import numpy as np
import pyquaternion

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

def R2D(rad):
    return rad*180 / np.pi

def add_input_noise(u0,model):
    # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)
    T = np.array([u0[0]])
    q = u0[1:]
    roll, pitch, yaw = quaternion_to_euler(q)
    mean = 0
    std_T = 0.01
    std_Angles = 0.01
    
    # std_q = np.std(q)
    T_noisy = T + np.random.normal(mean, std_T)
    T_noisy = max(min(T_noisy ,model.throttle_max), model.throttle_min)

    roll_noisy = roll + np.random.normal(mean, std_Angles)
    pitch_noisy = pitch + np.random.normal(mean, std_Angles)
    yaw_noisy = yaw + np.random.normal(mean, std_Angles)

    q_noisy = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)


    #for i, ui in enumerate(q):
    #    q[i] = ui + np.random.normal(mean, std_q)
    
    # create new noisy input vector
    u_noisy = np.append(T_noisy,q_noisy)

    return u_noisy