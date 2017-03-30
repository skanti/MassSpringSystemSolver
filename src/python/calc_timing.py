import matplotlib.pyplot as plt
import numpy as np


A1 = np.loadtxt("./pool300-langevin0.0001/pd2-langevin0.000100-timing.dat", skiprows=1)
A2 = np.loadtxt("./pool300-langevin0.0001/pd4-langevin0.000100-timing.dat", skiprows=1)
A3 = np.loadtxt("./pool300-langevin0.0001/pd8-langevin0.000100-timing.dat", skiprows=1)
A4 = np.loadtxt("./pool300-langevin0.0001/pd16-langevin0.000100-timing.dat", skiprows=1)
A5 = np.loadtxt("./pool300-langevin0.0001/pd32-langevin0.000100-timing.dat", skiprows=1)
A6 = np.loadtxt("./pool300-langevin0.0001/pd64-langevin0.000100-timing.dat", skiprows=1)
A7 = np.loadtxt("./pool300-langevin0.0001/newton1-langevin0.000100-timing.dat", skiprows=1)
A8 = np.loadtxt("./pool300-langevin0.0001/newton2-langevin0.000100-timing.dat", skiprows=1)
A9 = np.loadtxt("./pool300-langevin0.0001/newton5-langevin0.000100-timing.dat", skiprows=1)


plt.subplot(1,2,1)
plt.plot(A1, label="pd-2")
plt.plot(A2, label="pd-4")
plt.plot(A3, label="pd-8")
plt.plot(A4, label="pd-16")
plt.plot(A5, label="pd-32")
plt.plot(A6, label="pd-64")
plt.legend()
plt.ylim(0, 0.5)

plt.subplot(1,2,2)
plt.plot(A7, label="newton-1")
plt.plot(A8, label="newton-2")
plt.plot(A9, label="newton-5")
plt.legend()
plt.ylim(0, 0.5)

plt.show()
#print(np.shape(C))
