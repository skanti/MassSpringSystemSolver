import matplotlib.pyplot as plt
import numpy as np


A1 = np.loadtxt("newton1-timing0.dat")
A2 = np.loadtxt("newton2-timing0.dat")
A3 = np.loadtxt("newton5-timing0.dat")

A4 = np.loadtxt("pd2-timing0.dat")
A5 = np.loadtxt("pd5-timing0.dat")
A6 = np.loadtxt("pd10-timing0.dat")

A7 = np.loadtxt("newton1-langevin-timing0.dat")
A8 = np.loadtxt("newton2-langevin-timing0.dat")
A9 = np.loadtxt("newton5-langevin-timing0.dat")

A10 = np.loadtxt("pd2-langevin-timing0.dat")
A11 = np.loadtxt("pd5-langevin-timing0.dat")
A12 = np.loadtxt("pd10-langevin-timing0.dat")

plt.subplot(2, 2, 1)
plt.plot(A1, label="Newton 1-itr")
plt.plot(A2, label="Newton 2-itr")
plt.plot(A3, label="Newton 5-itr")
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(A4, label="PD 2-itr")
plt.plot(A5, label="PD 5-itr")
plt.plot(A6, label="PD 10-itr")
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(A7, label="Newton langevin 1-itr")
plt.plot(A8, label="Newton langevin 2-itr")
plt.plot(A9, label="Newton langevin 5-itr")
plt.legend()


plt.subplot(2, 2, 4)
plt.plot(A10, label="PD langevin 2-itr")
plt.plot(A11, label="PD langevin 5-itr")
plt.plot(A12, label="PD langevin 10-itr")
plt.legend()

plt.show()
