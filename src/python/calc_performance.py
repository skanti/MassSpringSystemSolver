import matplotlib.pyplot as plt
import numpy as np

n = 50*50
A0 = [] 
A1 = [] 
A = np.loadtxt("./pool300-langevin0.000000/newton20-langevin0.000000-traj.dat").reshape((-1, n, 3))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd2-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd4-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd8-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd16-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd32-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A0.append(np.loadtxt("./pool300-langevin0.000000/pd64-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A1.append(np.loadtxt("./pool300-langevin0.000000/newton1-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A1.append(np.loadtxt("./pool300-langevin0.000000/newton2-langevin0.000000-traj.dat").reshape((-1, n, 3)))
A1.append(np.loadtxt("./pool300-langevin0.000000/newton5-langevin0.000000-traj.dat").reshape((-1, n, 3)))

B0 = [] 
B1 = [] 
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd2-langevin0.000000-timing.dat")))
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd4-langevin0.000000-timing.dat")))
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd8-langevin0.000000-timing.dat")))
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd16-langevin0.000000-timing.dat")))
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd32-langevin0.000000-timing.dat")))
B0.append(np.median(np.loadtxt("./pool300-langevin0.000000/pd64-langevin0.000000-timing.dat")))
B1.append(np.median(np.loadtxt("./pool300-langevin0.000000/newton1-langevin0.000000-timing.dat")))
B1.append(np.median(np.loadtxt("./pool300-langevin0.000000/newton2-langevin0.000000-timing.dat")))
B1.append(np.median(np.loadtxt("./pool300-langevin0.000000/newton5-langevin0.000000-timing.dat")))

C0 = np.sum(np.linalg.norm(A0 - A, axis=3), axis=(1,2))
C1 = np.sum(np.linalg.norm(A1 - A, axis=3), axis=(1,2))
e_max = np.amax(np.append(C0, C1))
C0 = C0/e_max
C1 = C1/e_max

fig, ax0 = plt.subplots(1,1)
fig.suptitle("langevin-0.000000")

ax0.step(B0, C0, c='g', where='post', label="pd")
ax0.step(B1, C1, c='r', where='post', label="newton")
ax0.scatter(B0, C0, c='g')
ax0.scatter(B1, C1, c='r')
ax0.set_ylim(0)
ax0.set_xlim(0)
ax0.legend()
ax0.set_xlabel("time (optimization) / ms")
ax0.set_ylabel("normalized node distance error")

plt.show()

