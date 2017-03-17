import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


A = np.loadtxt("newton10-traj0.dat")
# A = A[-50*50:,:]
A = A[-50*50:,:]

print(np.shape(A))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(A[:, 2], A[:, 0], A[:, 1], alpha=0.5)

plt.show()
