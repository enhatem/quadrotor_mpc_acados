import matplotlib.pyplot as plt
import numpy as np

def plotRes(simX,simU,t, save=False):
    # plot results

    plt.style.use('seaborn')
    fig, (ax1,ax2,ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)

    ax1.step(t, simU[:,0], label = 'T')
    ax2.plot(t, simX[:,0], label='pz')
    ax3.plot(t, simX[:,1], label='vz')

    ax1.set_title('closed-loop simulation')
    ax1.set_ylabel('T [N]')
    ax1.legend()

    ax2.set_ylabel('pz[m]')
    ax2.legend()

    ax3.set_ylabel('vz[m]')
    ax3.set_xlabel('t[s]')
    ax3.legend()

    if save==True:
        fig.savefig('figures/statesAndInputs.png', dpi=300)