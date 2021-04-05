import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

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


def plotSim_pos(t, simX, ref_traj, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    
    ax1.plot(t, simX[1:,0], label='x_sim')
    ax1.plot(t, ref_traj[:,0], '--', label='x_ref')
    #ax1.plot(t, x_ref, '--', label ='x_ref')
    ax2.plot(t, simX[1:,1], label='y_sim')
    ax2.plot(t, ref_traj[:,1], '--', label='y_ref')
    #ax2.plot(t, y_ref, '--', label='x_ref')
    ax3.plot(t, simX[1:,2], label='z_sim')
    ax3.plot(t, ref_traj[:,2], '--', label='z_ref')
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

def plotSim_Angles(t, simX, simEuler, save=True):
    plt.style.use('seaborn')
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)

    ax1.plot(t, simX[1:,3], label='qw')
    ax1.plot(t, simX[1:,4], label='qx')
    ax1.plot(t, simX[1:,5], label='qy')
    ax1.plot(t, simX[1:,6], label='qz')

    ax2.plot(t,simEuler[1:,0], label='phi')
    ax2.plot(t,simEuler[1:,1], label='theta')
    ax2.plot(t,simEuler[1:,2], label='psi')

    ax1.set_title('States: Angles')
    ax1.set_ylabel('q')
    ax1.legend()

    ax2.set_ylabel('Euler angles [deg]')
    ax2.set_xlabel('t[s]')
    ax2.legend()

    if save ==True:
        fig.savefig('figures/anguleStates.png')

def plotSim_vel(t, simX, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    ax1.plot(t, simX[1:,7], label='vx')
    ax2.plot(t, simX[1:,8], label='vy')
    ax3.plot(t, simX[1:,9], label='vz')
    
    ax1.legend()
    ax1.set_title('States: linear velocities')
    ax1.set_ylabel('vx[m/s]')

    ax2.legend()
    ax2.set_ylabel('vy[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('vz[m/s]')
    
    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/velStates.png')


def plotThrustInput(t,simU,save=False):
    
    plt.style.use('seaborn')
    fig, ax1 = plt.subplots()

    ax1.step(t,simU[:,0], label='Thrust')
    ax1.legend()
    ax1.set_title('Control inputs: Thrust')
    ax1.set_xlabel('t[s]')
    ax1.set_ylabel('T[N]')

    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/thrustInput.png')

def plotAngularRatesInputs(t, simU, save=True):
    plt.style.use('seaborn')
    fig, (ax1,ax2,ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)
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
        fig.savefig('figures/thrustInputs.png')

def plotSim3D(simX, ref_traj, save=False):
    # 3D plot of the simulation
        # plotting the prediction trajectory done by the drone
    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    plt.title('3D trajectories')
    ax.plot3D(simX[:,0], simX[:,1], simX[:,2], label='sim')
    ax.plot3D(ref_traj[:,0], ref_traj[:,1], ref_traj[:,2], '--', label='ref')
    
    ax.legend()
    ax.set_xlabel('x[m]')
    ax.set_ylabel('y[m]')
    ax.set_zlabel('z[m]')

    plt.tight_layout()

    if save ==True:
        fig.savefig('figures/sim3D.png')


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

