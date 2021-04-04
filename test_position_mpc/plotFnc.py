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

def plotRes3D(simX,simU, simU_euler,t):
    # plot results
    
    '''
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    # plt.step(t, simU[:,1], color='g')
    plt.title('closed-loop simulation')
    plt.legend(['T'])
    plt.ylabel('u [N]')
    plt.xlabel('t [s]')
    plt.grid(True)
    '''
    # plotting the states
    plt.figure()
    plt.title('closed-loop simulation: States')
    plt.subplot(2, 3, 1)
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
    plt.plot(t, simU[:,0])
    plt.ylabel('T [N]')
    plt.xlabel('t [s]')
    plt.legend('T')
    plt.grid(True)
    plt.subplot(2,1,2)
    plt.plot(t, simX[:,1:])
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
    ax = plt.gca()  # or any other way to get an axis object
    ax.plot(t, simU_euler[:,0], label=r'$\theta$ [deg]')
    ax.legend()
    # plt.legend(r'$\phi$ [deg]')
    plt.grid(True)
    plt.subplot(1,3,2)
    plt.plot(t, simU_euler[:,1])
    plt.ylabel(r'$\theta$ [deg]')
    plt.xlabel('t [s]')
    plt.legend(r'$\theta$ [deg]')
    plt.grid(True)
    plt.subplot(1,3,3)
    plt.plot(t, simU_euler[:,2])
    plt.ylabel(r'$\psi$ [deg]')
    plt.xlabel('t [s]')
    plt.legend(r'$\psi$ [deg]')
    plt.grid(True)

    # plotting the trajectory done by the drone
    plt.figure()
    ax2 = plt.axes(projection = "3d")
    plt.title('Position trajectory')
    x = simX[:,0]
    y = simX[:,1]
    z = simX[:,2]
    ax2.plot3D(x, y, z)
