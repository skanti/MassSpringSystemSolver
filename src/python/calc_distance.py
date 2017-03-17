import matplotlib.pyplot as plt
import numpy as np


A = np.loadtxt("pd5-traj00.dat")
A = A.reshape((60, 40*40, 3))

B = np.loadtxt("newton20-traj00.dat")
B = B.reshape((60, 40*40, 3))

C = A[:] - B[:]
C = np.linalg.norm(C, axis=2)
C = np.sum(C, 1)

print(np.sum(C))

plt.plot(C)
plt.savefig("error.png")
plt.show()
#print(np.shape(C))