import numpy as np

import pandas as pd
import matplotlib.pyplot as plt

from extended_kalman_filter.ekf import EKF

def setup_ekf(DT, m, Ixx, x0, nx, nu):

    # C and D constant matrices
    C = np.eye(nx)
    D = np.zeros((nx,nu))

    # initial state and covariance
    x0 = x0
    P0 = np.eye(nx)
    P0[0][0] = 1e-2 # variance on the initial y
    P0[1][1] = 1e-2 # variance on the initial z
    P0[2][2] = 1e-2 # variance on the initial phi
    P0[3][3] = 1e-2 # variance on the initial vy
    P0[4][4] = 1e-2 # variance on the initial vz
    P0[5][5] = 1e-2 # variance on the initial phi_dot

    # variance of state noise in the prediction
    Q_alpha = np.zeros((6,6))
    Q_alpha[0][0] = 5e0  # variance of the state noise on y
    Q_alpha[1][1] = 5e-7  # variance of the state noise on z
    Q_alpha[2][2] = 1e0  # variance of the state noise on phi
    Q_alpha[3][3] = 1e-2  # variance of the state noise on vy
    Q_alpha[4][4] = 1e-2  # variance of the state noise on vz
    Q_alpha[5][5] = 1e0  # variance of the state noise on phi_dot

    # variance of the input noise
    Q_beta =  np.eye(nu)
    Q_beta[0][0] = (1e-3)**2 # variance of the noise on the thrust input u1 (unit: N)
    Q_beta[1][1] = (1e-4)**2 # variance of the noise on the torque input u2 (unit: N.m)

    # variance of the measurement noise (same values used in the magnitudes for state measurement noise)
    Q_gamma = np.eye(nx)
    Q_gamma[0][0] = 0.01**2 # variance on the y measurement (between -1cm and +1 cm)
    Q_gamma[1][1] = 0.01**2 # variance on the z measurement (between -1cm and +1 cm)
    Q_gamma[2][2] = 0.05**2 # variance on the phi measurement (between -5 degrees and +5 degrees)
    Q_gamma[3][3] = 0.01**2 # variance on the vy measurement (between -0.1m/s and +0.1m/s)
    Q_gamma[4][4] = 0.01**2 # variance on the vz measurement (between -0.1m/s and +0.1m/s)
    Q_gamma[5][5] = 0.05**2 # variance on the phi_dot measurement (between -5 degrees/s^2 and +5 degrees/s^2)


    ekf = EKF(initial_x=x0, P_0=P0, m=m, Ixx=Ixx, dt = DT)
    
    return ekf, C, D, Q_alpha, Q_beta, Q_gamma

    '''
    NUM_STEPS = simU.shape[0]
    MEAS_EVERY_STEPS = 1

    # defining the lists for the data to be stored at each iteration
    states  = []
    pred    = []
    covs    = []
    meas_xs = []

    # setting the seed for the input noise
    np.random.seed(20)

    # for loop to estimate the states
    for step in range(NUM_STEPS):

        phi_k   = float (ekf.state[2])  # roll at time k
        u_k     = simU[step,:]  # control input vector at time instant k
        var_u1 = Q_beta[0][0]
        var_u2 = Q_beta[1][1]
        noise   = np.array([np.random.normal(0, var_u1), np.random.normal(0, var_u2)])
        u_k     = u_k + noise   # adding input noise
        u1_k    = u_k[0]        # thrust control input at time instant k
        u2_k    = u_k[1]        # torque control input at time instant k

        # stacking the covariance matrix and the state estimation at each time instant
        covs.append(ekf.cov)
        states.append(ekf.state)

        # The measurement vector at each time instant
        meas_x = simX[step,:]

        # calculating the Jacobians of the evolution model with respect to the state vector and the input vector
        A_k = np.array([
            [1, 0,                                   0,  ekf._dt,       0,       0],
            [0, 1,                                   0,        0, ekf._dt,       0],
            [0, 0,                                   1,        0,       0, ekf._dt],
            [0, 0, -u1_k / m * np.cos(phi_k) * ekf._dt,        1,       0,       0],
            [0, 0, -u1_k / m * np.sin(phi_k) * ekf._dt,        0,       1,       0],
            [0, 0,                                   0,        0,       0,       1]
        ])

        B_k = np.array([
            [0,                                 0],
            [0,                                 0],
            [0,                                 0],
            [-np.sin(phi_k) / ekf._m * ekf._dt, 0],
            [ np.cos(phi_k) / ekf._m * ekf._dt, 0],
            [0,                ekf._dt / ekf._Ixx]
        ])

        # prediction phase
        ekf.predict(A = A_k, B = B_k, u = u_k, Q_alpha = Q_alpha, Q_beta = Q_beta)
        pred.append(ekf.state)

        # correction phase
        if step != 0 and step % MEAS_EVERY_STEPS == 0:
            ekf.update(C=C, meas=meas_x,
                    meas_variance=Q_gamma)
        meas_xs.append(meas_x)

    # converting lists to np arrays for plotting
    meas_xs = np.array(meas_xs)
    states = np.array(states)
    covs = np.array(covs)
    pred = np.array(pred)

    # extracting the y and z positions from the kalman filter estimation
    y_kalman        = states[:,0,0]
    z_kalman        = states[:,1,0]
    phi_kalman      = states[:,2,0] 
    vy_kalman       = states[:,3,0]
    vz_kalman       = states[:,4,0]
    phi_dot_kalman  = states[:,5,0]

    # extracting the variances of y and z for plotting the lower and upper bounds of the confidence interval 
    y_cov       = covs[:,0,0] # variance of y at each time instant
    z_cov       = covs[:,1,1] # variance of z at each time instant
    phi_cov     = covs[:,2,2] # variance of phi at each time instant
    vy_cov      = covs[:,3,3] # variance of vy at each time instant
    vz_cov      = covs[:,4,4] # variance of vz at each time instant
    phi_dot_cov = covs[:,5,5] # variance of phi_dot at each time instant

    # lower bound of confidence interval of the position (95%)
    lower_conf_y        = y_kalman - 2*np.sqrt(y_cov)
    lower_conf_z        = z_kalman - 2*np.sqrt(z_cov)
    lower_conf_phi      = phi_kalman - 2*np.sqrt(phi_cov)
    lower_conf_vy       = vy_kalman - 2*np.sqrt(vy_cov)
    lower_conf_vz       = vz_kalman - 2*np.sqrt(vz_cov)
    lower_conf_phi_dot  = phi_dot_kalman - 2*np.sqrt(phi_dot_cov)


    # lower bound of confidence interval of the position (95%)
    upper_conf_y        = y_kalman + 2*np.sqrt(y_cov)
    upper_conf_z        = z_kalman + 2*np.sqrt(z_cov)
    upper_conf_phi      = phi_kalman + 2*np.sqrt(phi_cov)
    upper_conf_vy       = vy_kalman + 2*np.sqrt(vy_cov)
    upper_conf_vz       = vz_kalman + 2*np.sqrt(vz_cov)
    upper_conf_phi_dot  = phi_dot_kalman + 2*np.sqrt(phi_dot_cov)

    # plotting the data
    fig1, ax1 = plt.subplots()

    ax1.set_title('Position')

    # plotting the measurements
    ax1.plot(meas_xs[:,0],meas_xs[:,1], label='real')

    # plotting the states after the prediction phase
    ax1.plot(pred[:,0], pred[:,1], '--', label='prediction')

    # plotting the extended kalman filter estimation
    ax1.plot(y_kalman, z_kalman, '--', label='kalman')

    fig2, (ax2, ax3, ax4) = plt.subplots(nrows = 3, ncols= 1, sharex = True)

    ax1.set_title('Performed trajectory')
    ax1.legend()

    # plotting confidence levels (commented out because weird results are obtained)  

    ax2.plot(meas_xs[:,0], label='y_meas')
    ax2.plot(pred[:,0], label='y_pred')
    ax2.plot(y_kalman, label='y_kalman')
    ax2.plot(lower_conf_y,'r--', label='lower bound confidence')
    ax2.plot(upper_conf_y,'r--', label='upper bound confidence')

    ax3.plot(meas_xs[:,1], label='z_meas')
    ax3.plot(pred[:,1], label='z_pred')
    ax3.plot(z_kalman, label='z_kalman')
    ax3.plot(lower_conf_z,'r--', label='lower bound confidence')
    ax3.plot(upper_conf_z,'r--', label='upper bound confidence')

    ax4.plot(meas_xs[:,2], label='phi_meas')
    ax4.plot(pred[:,2], label='phi_pred')
    ax4.plot(phi_kalman, label='phi_kalman')
    ax4.plot(lower_conf_phi,'r--', label='lower bound confidence')
    ax4.plot(upper_conf_phi,'r--', label='upper bound confidence')

    ax2.set_title('States: positions')
    ax2.legend()
    ax3.legend()
    ax4.legend()



    fig2, (ax5, ax6, ax7) = plt.subplots(nrows = 3, ncols= 1, sharex = True)

    ax5.plot(meas_xs[:,3], label='vy_meas')
    ax5.plot(pred[:,3], label='vy_pred')
    ax5.plot(vy_kalman, label='vy_kalman')
    ax5.plot(lower_conf_vy,'r--', label='lower bound confidence')
    ax5.plot(upper_conf_vy,'r--', label='upper bound confidence')

    ax6.plot(meas_xs[:,4], label='vz_meas')
    ax6.plot(pred[:,4], label='vz_pred')
    ax6.plot(vz_kalman, label='vz_kalman')
    ax6.plot(lower_conf_vz,'r--', label='lower bound confidence')
    ax6.plot(upper_conf_vz,'r--', label='upper bound confidence')

    ax7.plot(meas_xs[:,5], label='phi_dot_meas')
    ax7.plot(pred[:,5], label='phi_dot_pred')
    ax7.plot(phi_dot_kalman, label='phi_dot_kalman')
    ax7.plot(lower_conf_phi_dot,'r--', label='lower bound confidence')
    ax7.plot(upper_conf_phi_dot,'r--', label='upper bound confidence')

    ax5.set_title('States: velocities')
    ax5.legend()
    ax6.legend()
    ax7.legend()


    plt.show()
    '''