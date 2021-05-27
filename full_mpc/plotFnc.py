import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np


def plotDrone3D(ax,X,q):
    
    l= 0.046 # arm length
    r = 0.02 # rotor length

    x = X[0]
    y = X[1]
    z = X[2]

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    R = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])

    c1 = np.array([x,y,z]) + R @ np.array([r,0,0])
    q1 = np.array([x,y,z]) + R @ np.array([l,l,0])
    q2 = np.array([x,y,z]) + R @ np.array([-l,-l,0])
    q3 = np.array([x,y,z]) + R @ np.array([l,-l,0])
    q4 = np.array([x,y,z]) + R @ np.array([-l,l,0])

    r1 = q1 + R @ np.array([0,0,r])
    r2 = q2 + R @ np.array([0,0,r])
    r3 = q3 + R @ np.array([0,0,r])
    r4 = q4 + R @ np.array([0,0,r])

    ax.plot3D([q1[0], q2[0]], [q1[1], q2[1]], [q1[2], q2[2]], 'k')
    ax.plot3D([q3[0], q4[0]], [q3[1], q4[1]], [q3[2], q4[2]], 'k')
    ax.plot3D([q1[0], r1[0]], [q1[1], r1[1]], [q1[2], r1[2]], 'r')
    ax.plot3D([q2[0], r2[0]], [q2[1], r2[1]], [q2[2], r2[2]], 'r')
    ax.plot3D([q3[0], r3[0]], [q3[1], r3[1]], [q3[2], r3[2]], 'r')
    ax.plot3D([q4[0], r4[0]], [q4[1], r4[1]], [q4[2], r4[2]], 'r')
    ax.plot3D([x, c1[0]], [y, c1[1]], [z, c1[2]], '-', color='orange', label='heading')


def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)



def plotRes(simX,simU,t):
    # plot results
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    # plt.step(t, simU[:,1], color='g')
    plt.title('closed-loop simulation')
    plt.legend(['T'])
    plt.ylabel('u [N]')
    plt.xlabel('t [s]')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,:])
    plt.ylabel('x')
    plt.xlabel('t [s]')
    plt.legend(['pz','vz'])
    plt.grid(True)


def plotTraj(x,y,z):
    plt.style.use('seaborn')
    ax = plt.axes(projection = "3d")
    plt.title('Reference trajectory')
    ax.plot3D(x, y, z)
    #plt.tight_layout()
    plt.show()

def plotStates(t, simX,predX, x, y, z, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    x_pred = predX[:,0]
    y_pred = predX[:,1]
    z_pred = predX[:,2]
    x_ref  = x
    y_ref  = y
    z_ref  = z
    ax1.plot(t, x_pred, label='x_pred')
    ax1.plot(t, x_ref, '--', label ='x_ref')
    ax2.plot(t, y_pred, label='y_pred')
    ax2.plot(t, y_ref, '--', label='x_ref')
    ax3.plot(t, z_pred, label='z_pred')
    ax3.plot(t, z_ref, '--', label='z_ref')
    
    ax1.legend()
    ax1.set_title('Trajectoties along each axis')
    ax1.set_ylabel('x[m]')

    ax2.legend()
    ax2.set_ylabel('y[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('z[m]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/states.png')

    plt.show()


def plotPred(predX,ref_traj):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    ax1.plot(predX[:,0], label='x_pred')
    ax1.plot(ref_traj[:,0], '--', label='x_ref')
    #ax1.plot(t, x_ref, '--', label ='x_ref')
    ax2.plot(predX[:,1], label='y_pred')
    ax2.plot(ref_traj[:,1], '--', label='y_ref')
    #ax2.plot(t, y_ref, '--', label='x_ref')
    ax3.plot(predX[:,2], label='z_pred')
    ax3.plot(ref_traj[:,2], '--', label='z_ref')
    #ax3.plot(t, z_ref, '--', label='z_ref')

    ax1.legend()
    ax1.set_title('Predicted Trajectoties along each axis')
    ax1.set_yticks([-1,0,1])
    ax1.set_ylabel('x[m]')

    ax2.legend()
    ax2.set_yticks([-1,0,1])
    ax2.set_ylabel('y[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('z[m]')
    
    plt.tight_layout()
    '''
    if save ==True:
        fig.savefig('figures/states.png')
    '''
    plt.show()


def plotSim_pos(t, simX, ref_traj, Nsim, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]
    
    ax1.plot(t, simX[1:,0], label='x_sim')
    ax1.plot(t, ref_traj[:Nsim,0], '--', label='x_ref')
    #ax1.plot(t, x_ref, '--', label ='x_ref')
    ax2.plot(t, simX[1:,1], label='y_sim')
    ax2.plot(t, ref_traj[:Nsim,1], '--', label='y_ref')
    #ax2.plot(t, y_ref, '--', label='x_ref')
    ax3.plot(t, simX[1:,2], label='z_sim')
    ax3.plot(t, ref_traj[:Nsim,2], '--', label='z_ref')
    #ax3.plot(t, z_ref, '--', label='z_ref')
    
    ax1.legend()
    ax1.set_title('States: Positions')
    ax1.set_ylabel('px[m]')

    ax2.legend()
    ax2.set_ylabel('py[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('pz[m]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/posStates.png')




def plotSim_pos_ref_point(t, simX, Nsim, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,0], label='x_sim')
    ax2.plot(t, simX[1:,1], label='y_sim')
    ax3.plot(t, simX[1:,2], label='z_sim')
    
    ax1.legend()
    ax1.set_title('States: Positions')
    ax1.set_ylabel('px[m]')

    ax2.legend()
    ax2.set_ylabel('py[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('pz[m]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/posStates.png')



def plotSim3D(simX, ref_traj, save=False):
    
    # extracting initial position
    # x_start = simX[0][0]
    # y_start = simX[0][1]
    # z_start = simX[0][2]

    # extracting final position
    # x_end = simX[-1][0]
    # y_end = simX[-1][1]
    # z_end = simX[-1][2]

    plt.style.use('seaborn')

    x = simX[:,0]
    y = simX[:,1]
    z = simX[:,2]

    x_ref = ref_traj[:,0]
    y_ref = ref_traj[:,1]
    z_ref = ref_traj[:,2]

    fig, ax = plt.subplots()
    plt.title('Reference trajectory')
    ax = plt.axes(projection = "3d")
    ax.plot3D(x, y, z, label='meas')
    ax.plot3D(x_ref, y_ref, z_ref, label='ref')
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    # fig, ax = plt.subplots()

    # ax.plot(simX[:,0], simX[:,1], label='traj')
    # ax.plot(ref_traj[0:Nsim,0], ref_traj[0:Nsim,1], '--', label='ref_traj')

    # plotting initial and final position
    # ax.plot(y_start,z_start,'.', color='red')
    # ax.text(y_start,z_start,'start', color='red')
    # ax.plot(y_end,z_end,'.', color = 'red')
    # ax.text(y_end,z_end,'end', color='red')
    
    ax.legend()
    ax.set_title("Performed Trajectory")
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    # ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))  # aspect ratio is 1:1:1 in data space


    NUM_STEPS = simX.shape[0]
    MEAS_EVERY_STEPS = 40

    X0 = [simX[0,0], simX[0,1], simX[0,2]]
    q0 = [simX[0,3], simX[0,4], simX[0,5], simX[0,6]]
    plotDrone3D(ax,X0,q0)
    
    for step in range(NUM_STEPS):
        if step !=0 and step % MEAS_EVERY_STEPS ==0:
            X = [simX[step,0], simX[step,1], simX[step,2]]
            q = [simX[step,3], simX[step,4], simX[step,5], simX[step,6]]
            plotDrone3D(ax,X,q)

    axisEqual3D(ax)
    

    if save == True:
        fig.savefig('figures/sim3D.png')




























def plotSim_Angles(t, simX, simEuler, Nsim, save=True):
    plt.style.use('seaborn')
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,3], label='qw')
    ax1.plot(t, simX[1:,4], label='qx')
    ax1.plot(t, simX[1:,5], label='qy')
    ax1.plot(t, simX[1:,6], label='qz')

    ax2.plot(t,simEuler[1:,0], label='phi')
    ax2.plot(t,simEuler[1:,1], label='theta')
    ax2.plot(t,simEuler[1:,2], label='psi')

    ax1.set_title('States: Angles')
    ax1.set_ylabel('quaternions')
    ax1.legend()

    ax2.set_ylabel('Euler angles [deg]')
    ax2.set_xlabel('t[s]')
    ax2.legend()

    if save ==True:
        fig.savefig('figures/angleStates.png')

def plotSim_vel(t, simX, Nsim, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,7], label='vx')
    ax2.plot(t, simX[1:,8], label='vy')
    ax3.plot(t, simX[1:,9], label='vz')
    
    ax1.legend()
    ax1.set_title('States: Linear velocities')
    ax1.set_ylabel('vx[m/s]')

    ax2.legend()
    ax2.set_ylabel('vy[m/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('vz[m/s]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/velStates.png')


def plotSim_vel_with_ref(t, simX, ref_traj, Nsim, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    ref_traj = ref_traj[0:Nsim]

    vx_ref = ref_traj[:,3]
    vy_ref = ref_traj[:,4]
    vz_ref = ref_traj[:,5]

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,7], label='vx')
    ax1.plot(t, vx_ref, '--', label='vx_ref')
    ax2.plot(t, simX[1:,8], label='vy')
    ax2.plot(t, vy_ref, '--', label='vy_ref')
    ax3.plot(t, simX[1:,9], label='vz')
    ax3.plot(t, vz_ref, '--', label='vz_ref')
    
    ax1.legend()
    ax1.set_title('States: Linear velocities')
    ax1.set_ylabel('vx[m/s]')

    ax2.legend()
    ax2.set_ylabel('vy[m/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('vz[m/s]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/velStates.png')



def plotThrustInput(t, simU, Nsim, save=False):
    
    plt.style.use('seaborn')
    fig, ax1 = plt.subplots()

    t = t[0:Nsim]

    ax1.step(t,simU[:,0], label='Thrust')
    ax1.legend()
    ax1.set_title('Control inputs: Thrust')
    ax1.set_xlabel('t[s]')
    ax1.set_ylabel('T[N]')

    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/thrustInput.png')

def plotAngularRatesInputs(t, simU, Nsim, save=True):
    plt.style.use('seaborn')
    fig, (ax1,ax2,ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)
    
    t = t[0:Nsim]

    ax1.step(t,simU[:,1], label='wx')
    ax2.step(t,simU[:,2], label='wy')
    ax3.step(t,simU[:,3], label='wz')
    
    ax1.legend()
    ax1.set_title('Control inputs: Angular Rates')
    ax1.set_ylabel('wx[rad/s]')

    ax2.legend()
    ax2.set_ylabel('wy[rad/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('wz[rad/s]')

    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/angulareRatesInputs.png')

def plotAngleInputs(t,simU,eulerAngles,save=False):

    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)    

    ax1.step(t,simU[:,1], label='qw')
    ax1.step(t,simU[:,2], label='qx')
    ax1.step(t,simU[:,3], label='qy')
    ax1.step(t,simU[:,4], label='qz')

    ax2.step(t,eulerAngles[:,0], label='phi')
    ax2.step(t,eulerAngles[:,1], label='theta')
    ax2.step(t,eulerAngles[:,2], label='psi')


    ax1.legend()
    ax1.set_ylabel('q')
    
    ax2.legend()
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('Angles[deg]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/angleInputs.png')


def plotRes3D(predX, simX, simU, simU_euler, t):
    # plot results

    # plotting the states
    plt.figure()
    plt.title('closed-loop simulation: States')
    plt.subplot(2, 3, 1)
    plt.plot(t, predX[:,0])
    plt.plot(t, simX[:,0])
    plt.ylabel('x')
    plt.xlabel('t [s]')
    plt.legend('px')
    plt.grid(True)

    plt.subplot(2, 3, 2)
    plt.plot(t, simX[:,1])
    plt.ylabel('y')
    plt.xlabel('t [s]')
    plt.legend('py')
    plt.grid(True)

    plt.subplot(2, 3, 3)
    plt.plot(t, simX[:,2])
    plt.ylabel('z')
    plt.xlabel('t [s]')
    plt.legend('pz')
    plt.grid(True)

    plt.subplot(2, 3, 4)
    plt.plot(t, simX[:,3])
    plt.ylabel('vx')
    plt.xlabel('t [s]')
    plt.legend('vx')
    plt.grid(True)

    plt.subplot(2, 3, 5)
    plt.plot(t, simX[:,4])
    plt.ylabel('vy')
    plt.xlabel('t [s]')
    plt.legend('vy')
    plt.grid(True)

    plt.subplot(2, 3, 6)
    plt.plot(t, simX[:,5])
    plt.ylabel('vz')
    plt.xlabel('t [s]')
    plt.legend('vz')
    plt.grid(True)

    # plotting the thrust
    plt.figure()
    plt.title('closed-loop simulation: Thrust')
    plt.subplot(2,1,1)
    plt.step(t, simU[:,0])
    plt.ylabel('T [N]')
    plt.xlabel('t [s]')
    plt.legend('T')
    plt.grid(True)
    plt.subplot(2,1,2)
    plt.step(t, simX[:,1:])
    plt.ylabel('q')
    plt.xlabel('t [s]')
    plt.legend(['qw', 'qx', 'qy', 'qz'])
    plt.grid(True)

    # plotting the euler angles
    plt.figure()
    plt.subplot(1,3,1)
    plt.title('closed-loop simulation: Angles')
    # plt.plot(t, simU_euler[:,0])
    plt.ylabel(r'$\phi$ [deg]')
    plt.xlabel('t [s]')
    ax = plt.gca()
    ax.step(t, simU_euler[:,0], label=r'$\phi$ [deg]')
    ax.legend()
    # plt.legend(r'$\phi$ [deg]')
    plt.grid(True)
    plt.subplot(1,3,2)
    #plt.step(t, simU_euler[:,1])
    plt.ylabel(r'$\theta$ [deg]')
    plt.xlabel('t [s]')
    ax2 = plt.gca()
    ax2.step(t, simU_euler[:,1], label=r'$\theta$ [deg]')
    ax2.legend
    #plt.legend(r'$\theta$ [deg]')
    plt.grid(True)
    plt.subplot(1,3,3)
    #plt.step(t, simU_euler[:,2])
    plt.ylabel(r'$\psi$ [deg]')
    plt.xlabel('t [s]')
    ax3 = plt.gca()
    ax3.step(t, simU_euler[:,2], label=r'$\psi$ [deg]')
    ax3.legend
    #plt.legend(r'$\psi$ [deg]')
    plt.grid(True)

    # plotting the prediction trajectory done by the drone
    plt.figure()
    ax4 = plt.axes(projection = "3d")
    plt.title('Position trajectory')
    x = simX[:,0]
    y = simX[:,1]
    z = simX[:,2]
    ax4.plot3D(x, y, z)

def plotSim3D_ref_point(simX, X_ref, save=False):

    plt.style.use('seaborn')

    x = simX[:,0]
    y = simX[:,1]
    z = simX[:,2]    

    x_ref = X_ref[0]
    y_ref = X_ref[1]
    z_ref = X_ref[2]

    fig, ax = plt.subplots()
    ax = plt.axes(projection = "3d")
    ax.plot3D(x, y, z, label='meas')
    ax.plot3D(x_ref, y_ref, z_ref, 'r.', label='goal')

    # fig, ax = plt.subplots()

    # ax.plot(simX[:,0], simX[:,1], label='traj')
    # ax.plot(ref_traj[0:Nsim,0], ref_traj[0:Nsim,1], '--', label='ref_traj')

    # plotting initial and final position
    # ax.plot(y_start,z_start,'.', color='red')
    # ax.text(y_start,z_start,'start', color='red')
    # ax.plot(y_end,z_end,'.', color = 'red')
    # ax.text(y_end,z_end,'end', color='red')

    ax.legend()
    ax.set_title("Performed Trajectory")
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")

    if save == True:
        fig.savefig('figures/sim3D.png')



def plotErrors_no_vel(t, simX, ref_traj, Nsim, save=False):
    # errors 
    ref_traj        = ref_traj[0:Nsim, :]
    t = t[0:Nsim]

    x_error         = ref_traj[:,0] - simX[1:,0]
    y_error         = ref_traj[:,1] - simX[1:,1]
    z_error         = ref_traj[:,2] - simX[1:,2]
    # vy_error        = ref_traj[:,2] - simX[1:,3]
    # vz_error        = ref_traj[:,3] - simX[1:,4]

    fig1, (ax1,ax2,ax3) = plt.subplots(nrows = 3, ncols = 1, sharex = True)

    ax1.plot(t, x_error, label='x_error')
    ax2.plot(t, y_error, label='y_error')
    ax3.plot(t, z_error, label='z_error')

    ax1.set_title('Errors: Position')
    ax1.set_ylabel('x_error[m]')

    ax2.set_ylabel('y_error[m]')

    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('z_error[m]')

    if save == True:
        fig1.savefig('figures/Errors_position.png', dpi=300)

def plotErrors_with_vel(t, simX, ref_traj, Nsim, save=False):
    # errors 
    ref_traj        = ref_traj[0:Nsim, :]
    t = t[0:Nsim]

    x_error         = ref_traj[:,0] - simX[1:,0]
    y_error         = ref_traj[:,1] - simX[1:,1]
    z_error         = ref_traj[:,2] - simX[1:,2]
    vx_error        = ref_traj[:,3] - simX[1:,7]
    vy_error        = ref_traj[:,4] - simX[1:,8]
    vz_error        = ref_traj[:,5] - simX[1:,9]

    fig1, (ax1,ax2,ax3) = plt.subplots(nrows = 3, ncols = 1, sharex = True)

    ax1.plot(t, x_error, label='x_error')
    ax2.plot(t, y_error, label='y_error')
    ax3.plot(t, z_error, label='z_error')

    ax1.legend()
    ax1.set_title('Errors: Position')
    ax1.set_ylabel('x_error[m]')

    ax2.legend()
    ax2.set_ylabel('y_error[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('z_error[m]')

    fig2, (ax4,ax5, ax6) = plt.subplots(nrows = 3, ncols = 1, sharex = True)

    ax4.plot(t, vx_error, label='vx_error')
    ax5.plot(t, vy_error, label='vy_error')
    ax6.plot(t, vz_error, label='vz_error')

    ax4.legend()
    ax4.set_title('Errors: Velocities')
    ax4.set_ylabel('vx_error[m/s]')

    ax5.legend()
    ax5.set_ylabel('vy_error[m/s]')

    ax6.legend()
    ax6.set_xlabel('t[s]')
    ax6.set_ylabel('vz_error[m/s]')

    if save == True:
        fig1.savefig('figures/Errors_position.png', dpi=300)
        fig2.savefig('figures/Errors_velocities.png', dpi=300)