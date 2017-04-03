import matplotlib.pyplot as plt
import numpy as np

n = 50*50
A0 = np.loadtxt("./pool300-langevin0/newton20-langevin0.000000-traj.dat").reshape((-1, n, 3))
A1 = np.loadtxt("./pool300-langevin0/pd2-langevin0.000000-traj.dat").reshape((-1, n, 3))
A2 = np.loadtxt("./pool300-langevin0/pd4-langevin0.000000-traj.dat").reshape((-1, n, 3))
A3 = np.loadtxt("./pool300-langevin0/pd8-langevin0.000000-traj.dat").reshape((-1, n, 3))
A4 = np.loadtxt("./pool300-langevin0/pd16-langevin0.000000-traj.dat").reshape((-1, n, 3))
A5 = np.loadtxt("./pool300-langevin0/pd32-langevin0.000000-traj.dat").reshape((-1, n, 3))
A6 = np.loadtxt("./pool300-langevin0/pd64-langevin0.000000-traj.dat").reshape((-1, n, 3))
A7 = np.loadtxt("./pool300-langevin0/newton1-langevin0.000000-traj.dat").reshape((-1, n, 3))
A8 = np.loadtxt("./pool300-langevin0/newton2-langevin0.000000-traj.dat").reshape((-1, n, 3))
A9 = np.loadtxt("./pool300-langevin0/newton5-langevin0.000000-traj.dat").reshape((-1, n, 3))

A1 = np.sum(np.linalg.norm(A1 - A0, axis=2), 1)
A2 = np.sum(np.linalg.norm(A2 - A0, axis=2), 1)
A3 = np.sum(np.linalg.norm(A3 - A0, axis=2), 1)
A4 = np.sum(np.linalg.norm(A4 - A0, axis=2), 1)
A5 = np.sum(np.linalg.norm(A5 - A0, axis=2), 1)
A6 = np.sum(np.linalg.norm(A6 - A0, axis=2), 1)
A7 = np.sum(np.linalg.norm(A7 - A0, axis=2), 1)
A8 = np.sum(np.linalg.norm(A8 - A0, axis=2), 1)
A9 = np.sum(np.linalg.norm(A9 - A0, axis=2), 1)


fig, (ax0, ax1) = plt.subplots(1,2, sharey=True)
fig.suptitle("langevin-0")

ax0.plot(A1, label="pd-2")
ax0.plot(A2, label="pd-4")
ax0.plot(A3, label="pd-8")
ax0.plot(A4, label="pd-16")
ax0.plot(A5, label="pd-32")
ax0.plot(A6, label="pd-64")
ax0.legend()
ax0.set_xlabel("# simulation iterations (by dt)")
ax0.set_ylabel("summed node distance error")

ax1.plot(A7, label="newton-1")
ax1.plot(A8, label="newton-2")
ax1.plot(A9, label="newton-5")
ax1.legend()
ax1.set_xlabel("# simulation iterations (by dt)")


plt.show()

