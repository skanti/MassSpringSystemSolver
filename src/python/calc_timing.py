import matplotlib.pyplot as plt
import numpy as np


A = np.loadtxt("timing.dat", skiprows=1, usecols=[0, 1])
A = A.reshape((2, 4, 2))
B = A[1]
A = A[0]


plt.loglog(A[:, 0], A[:, 1], label="Newton")
plt.loglog(B[:, 0], B[:, 1], label="variational")
plt.legend()
# plt.savefig("error.png")
plt.show()
#print(np.shape(C))