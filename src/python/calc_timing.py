import matplotlib.pyplot as plt
import numpy as np


A1 = np.loadtxt("./pool300-langevin0.000001/pd2-langevin0.000001-timing.dat", skiprows=1)
A2 = np.loadtxt("./pool300-langevin0.000001/pd4-langevin0.000001-timing.dat", skiprows=1)
A3 = np.loadtxt("./pool300-langevin0.000001/pd8-langevin0.000001-timing.dat", skiprows=1)
A4 = np.loadtxt("./pool300-langevin0.000001/pd16-langevin0.000001-timing.dat", skiprows=1)
A5 = np.loadtxt("./pool300-langevin0.000001/pd32-langevin0.000001-timing.dat", skiprows=1)
A6 = np.loadtxt("./pool300-langevin0.000001/pd64-langevin0.000001-timing.dat", skiprows=1)
A7 = np.loadtxt("./pool300-langevin0.000001/newton1-langevin0.000001-timing.dat", skiprows=1)
A8 = np.loadtxt("./pool300-langevin0.000001/newton2-langevin0.000001-timing.dat", skiprows=1)
A9 = np.loadtxt("./pool300-langevin0.000001/newton5-langevin0.000001-timing.dat", skiprows=1)

fig, (ax0, ax1) = plt.subplots(1,2, sharey=True)
fig.suptitle("timing langevin-0.000001")
ax0.plot(A1, label="pd-2")
ax0.plot(A2, label="pd-4")
ax0.plot(A3, label="pd-8")
ax0.plot(A4, label="pd-16")
ax0.plot(A5, label="pd-32")
ax0.plot(A6, label="pd-64")
ax0.set_xlabel("# simulation iterations (by dt)")
ax0.set_ylabel("summed node distance error")
ax0.legend()
ax0.set_ylim(0, 300)

ax1.plot(A7, label="newton-1")
ax1.plot(A8, label="newton-2")
ax1.plot(A9, label="newton-5")
ax1.set_xlabel("# simulation iterations (by dt)")
ax1.legend()

plt.show()
#print(np.shape(C))
