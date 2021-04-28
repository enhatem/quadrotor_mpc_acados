import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from utils import *
from matplotlib.animation import FuncAnimation
from itertools import count


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



def livePlot(simX):
    y = simX[:,0]
    z = simX[:,1]

    plt.style.use('fivethirtyeight')

    plt.cla()
    plt.plot(y, z, label='trajectory')

    plt.legend(loc = 'upper left')
    plt.tight_layout()
    
    index = count()


def plotPos_ref_point(t,simX, save=True):
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3) = plt.subplots(nrows= 3, ncols = 1, sharex=True)

    ax1.plot(t, simX[1:,0], label='y')
    ax2.plot(t, simX[1:,1], label='z')
    ax3.plot(t, R2D(simX[1:,2]), label='phi')
    
    ax1.legend()
    ax1.set_title('States: Positions')
    ax1.set_ylabel('py[m]')

    ax2.legend()
    ax2.set_ylabel('pz[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('phi[deg]')

    if save == True:
        fig.savefig('figures/posStates.png', dpi=300)

def plotDrone(ax,X_0,phi):
    
    L= 0.046
    l = 0.3 * L

    X1 = [X_0[0] + L*np.cos(phi),  X_0[1] + L*np.sin(phi)] # right arm of drone
    X2 = [X_0[0] + L*np.cos(phi + np.pi), X_0[1] + L*np.sin(phi + np.pi)] # left arm of drone
    X3 = [X1[0] +  l*np.cos(np.pi/2 + phi), X1[1] + l*np.sin(np.pi/2 + phi)] # upper right
    X4 = [X2[0] +  l*np.cos(phi + np.pi/2), X2[1] + l*np.sin(phi + np.pi/2)] # upper right

    # right arm
    x_values_1 = [X_0[0], X1[0]]
    y_values_1 = [X_0[1], X1[1]]

    # left arm
    x_values_2 = [X_0[0], X2[0]]
    y_values_2 = [X_0[1], X2[1]]

    # upper right arm
    x_values_3 = [X1[0], X3[0]]
    y_values_3 = [X1[1], X3[1]]

    # upper left arm
    x_values_4 = [X2[0], X4[0]]
    y_values_4 = [X2[1], X4[1]]

    ax.plot(x_values_1, y_values_1,'k')
    ax.plot(x_values_2, y_values_2,'k')
    ax.plot(x_values_3,y_values_3,'r')
    ax.plot(x_values_4,y_values_4,'r')



def plotSim_ref_point(simX, save=False):

    
    plt.style.use('seaborn')

    fig, ax = plt.subplots()

    ax.plot(simX[:,0], simX[:,1], label='traj')

    NUM_STEPS = simX.shape[0]
    MEAS_EVERY_STEPS = 15

    for step in range(NUM_STEPS):
        if step !=0 and step % MEAS_EVERY_STEPS ==0:
            phi = simX[step,2]
            X = [simX[step,0], simX[step,1]]
            plotDrone(ax,X,phi)

    ax.legend()
    ax.set_title("Performed Trajectory")
    ax.set_xlabel("y[m]")
    ax.set_ylabel("z[m]")

    if save == True:
        fig.savefig('figures/sim.png', dpi=300)

def plotSim(simX, ref_traj, Nsim, save=False):
    plt.style.use('seaborn')

    fig, ax = plt.subplots()

    ax.plot(simX[:,0], simX[:,1], label='traj')
    ax.plot(ref_traj[0:Nsim,0], ref_traj[0:Nsim,1], '--', label='ref_traj')

    ax.legend()
    ax.set_title("Performed Trajectory")
    ax.set_xlabel("y[m]")
    ax.set_ylabel("z[m]")

    NUM_STEPS = simX.shape[0]
    MEAS_EVERY_STEPS = 15

    X0 = [simX[0,0], simX[0,1]]
    phi_0 = simX[0,2]
    plotDrone(ax,X0,phi_0)
    
    for step in range(NUM_STEPS):
        if step !=0 and step % MEAS_EVERY_STEPS ==0:
            phi = simX[step,2]
            X = [simX[step,0], simX[step,1]]
            plotDrone(ax,X,phi)

    if save == True:
        fig.savefig('figures/sim.png', dpi=300)

def plotPos(t, simX, ref_traj, Nsim, save=False):
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3) = plt.subplots(nrows= 3, ncols = 1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,0], label='y')
    ax1.plot(t, ref_traj[0:Nsim,0], '--', label='y_ref')
    ax2.plot(t, simX[1:,1], label='z')
    ax2.plot(t, ref_traj[0:Nsim,1], '--', label='z_ref')
    ax3.plot(t, R2D(simX[1:,2]), label='phi')
    # ax3.plot(t, R2D(ref_traj[0:Nsim,2]), label='phi_ref')
    
    ax1.legend()
    ax1.set_title('States: Positions')
    ax1.set_ylabel('py[m]')

    ax2.legend()
    ax2.set_ylabel('pz[m]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('phi[deg]')

    if save == True:
        fig.savefig('figures/posStates.png', dpi=300)


def plotVel(t, simX, Nsim, save=False):
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,3], label='vy')
    ax2.plot(t, simX[1:,4], label='vz')
    ax3.plot(t, simX[1:,5], label='phi_dot')

    ax1.legend()
    ax1.set_title('States: Rates')
    ax1.set_ylabel('vy[m/s]')

    ax2.legend()
    ax2.set_ylabel('vz[m/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('phi_dot[rad/s]')

    if save == True:
        fig.savefig('figures/rateStates.png', dpi=300)

def plotVel_with_vy_vz_references(t,simX, ref_traj, Nsim, save=True):
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,3], label='vy')
    ax1.plot(t, ref_traj[0:Nsim,2], '--', label='ref_vy')
    ax2.plot(t, simX[1:,4], label='vz')
    ax2.plot(t, ref_traj[0:Nsim,3], '--', label='ref_vz')
    ax3.plot(t, simX[1:,5], label='phi_dot')

    ax1.legend()
    ax1.set_title('States: Rates')
    ax1.set_ylabel('vy[m/s]')

    ax2.legend()
    ax2.set_ylabel('vz[m/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('phi_dot[rad/s]')

    if save == True:
        fig.savefig('figures/rateStates.png', dpi=300)

def plotVel_with_ref(t, simX, ref_traj, Nsim, save=False):
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.plot(t, simX[1:,3], label='vy')
    ax1.plot(t, ref_traj[0:Nsim,3], '--', label='ref_vy')
    ax2.plot(t, simX[1:,4], label='vz')
    ax2.plot(t, ref_traj[0:Nsim,4], '--', label='ref_vz')
    ax3.plot(t, simX[1:,5], label='phi_dot')
    ax3.plot(t, ref_traj[0:Nsim,5], '--', label='ref_phi_dot')

    ax1.legend()
    ax1.set_title('States: Rates')
    ax1.set_ylabel('vy[m/s]')

    ax2.legend()
    ax2.set_ylabel('vz[m/s]')

    ax3.legend()
    ax3.set_xlabel('t[s]')
    ax3.set_ylabel('phi_dot[rad/s]')

    if save == True:
        fig.savefig('figures/rateStates.png', dpi=300)

def plotSimU_with_ref(t,simU,ref_U, Nsim, save=False):
    plt.style.use('seaborn')
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)

    ax1.step(t, simU[:,0], label='T')
    ax1.step(t, ref_U[0:Nsim,0], '--',label='T_ref')
    ax2.step(t, simU[:,1], label='Tau')
    ax2.step(t, ref_U[0:Nsim,1], '--', label='Tau_ref')

    ax1.legend()
    ax1.set_title('Control Inputs')
    ax1.set_ylabel('T[N]')

    ax2.legend()
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('Tau[N.m]')

    if save == True:
        fig.savefig('figures/controlInputs.png', dpi=300)

def plotSimU(t, simU, Nsim, save=False):
    plt.style.use('seaborn')
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)

    t = t[0:Nsim]

    ax1.step(t, simU[:,0], label='T')
    ax2.step(t, simU[:,1], label='Tau')

    ax1.legend()
    ax1.set_title('Control Inputs')
    ax1.set_ylabel('T[N]')

    ax2.legend()
    ax2.set_xlabel('t[s]')
    ax2.set_ylabel('Tau[N.m]')

    if save == True:
        fig.savefig('figures/controlInputs.png', dpi=300)

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
    ax1.set_ylabel('quaternions')
    ax1.legend()

    ax2.set_ylabel('Euler angles [deg]')
    ax2.set_xlabel('t[s]')
    ax2.legend()

    if save ==True:
        fig.savefig('figures/angleStates.png')

def plotSim_vel(t, simX, save=False):
    # figure: container holding the plots (can have multiple plots)
    # axes: actual plots
    plt.style.use('seaborn')
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

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
        fig.savefig('figures/angleInputs.png')

def plotSim3D(simX, ref_traj, save=False):
    # 3D plot of the simulation
        # plotting the prediction trajectory done by the drone
    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    plt.title('3D trajectory')
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

