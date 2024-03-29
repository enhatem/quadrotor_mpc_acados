import random
import numpy as np
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')
'''
x_vals = [0, 1, 2, 3, 4, 5]
y_vals = [0, 1, 3, 2, 3, 5]

# plt.plot(x_vals, y_vals)

index = count()

def animate(i):
    x_vals.append(next(index))
    y_vals.append(random.randint(0,5))
    plt.cla() # to clear the axis
    plt.plot(x_vals, y_vals)

ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()
'''

t = np.linspace(0,2*np.pi) 
a = 30 ; b = 15 
x = (a*np.cos(t) * np.cos(np.pi /4))  + 5
y = (b*np.sin(t)  * np.sin(np.pi /4))  + 4
plt.plot(x,y)
plt.plot(5,4,'r.')
plt.show()