# from pyplot3d.uav import Uav
# from pyplot3d.utils import ypr_to_R
# import matplotlib.pyplot as plt
# import numpy as np

# plt.style.use('seaborn')

# # initialize the plot 
# fig = plt.figure()
# ax = fig.gca(projection="3d")

# arm_length = 0.24
# uav = Uav(ax, arm_length=arm_length)
# uav.draw_at([1,0,0], ypr_to_R([np.pi/2.0,0,0]))
# plt.show()

import numpy as np
from matplotlib import pyplot as plt
# %matplotlib inline

from matplotlib import animation

from uav import pyplot3d.Uav
from utils import pyplot3d.ypr_to_R


def update_plot(i, x, R):
    uav_plot.update_plot(x[:, i], R[:, :, i])
    
    # These limits must be set manually since we use
    # a different axis frame configuration than the
    # one matplotlib uses.
    xmin, xmax = -2, 2
    ymin, ymax = -2, 2
    zmin, zmax = -2, 2
    
    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymax, ymin])
    ax.set_zlim([zmax, zmin])


plt.style.use('seaborn')

fig = plt.figure()
ax = fig.gca(projection='3d')

arm_length = 0.24  # in meters
uav_plot = Uav(ax, arm_length)


# Create some fake simulation data
steps = 60
t_end = 1

x = np.zeros((3, steps))
x[0, :] = np.arange(0, t_end, t_end / steps)
x[1, :] = np.arange(0, t_end, t_end / steps) * 2

R = np.zeros((3, 3, steps))
for i in range(steps):
    ypr = np.array([i, 0.1 * i, 0.0])
    R[:, :, i] = ypr_to_R(ypr, degrees=True)

ani = animation.FuncAnimation(fig, update_plot, frames=20, fargs=(x, R,));

# If using Jupyter Notebooks
# from IPython.display import HTML
# HTML(ani.to_jshtml())
