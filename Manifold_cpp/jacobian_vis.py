import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rc

which_twc= 7
_save_ext = ".png"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
J = np.loadtxt("../Data/Jcurve.csv", delimiter=",")
n= len(J)/16380 #number of direction samples.
J = np.array_split(J, n) #(16380, 1)

directions = np.loadtxt("../brute_force_xyz_indexes_two_degree.csv", delimiter=",") #(16380,3)
ax.scatter(directions[:, 0], directions[:, 1], directions[:, 2], 
                c=J[which_twc], alpha=0.4, cmap='viridis')

# ________________________________________

with open("../Data/Rwhileopt.csv", "r") as f:
    lines = f.read().strip().split("\n\n")
Rs = [np.loadtxt(line.splitlines()) for line in lines]
Rs= Rs[which_twc]
N=Rs.shape[0] // 3
r_vecs= Rs.reshape(N, 3)  
ax.scatter(r_vecs[:, 0], r_vecs[:, 1], r_vecs[:, 2], c=np.arange(len(r_vecs)), cmap="viridis", s=50, label="Optimization Steps")

updates = np.diff(r_vecs, axis=0) 
for i in range(len(updates)):
    ax.quiver(r_vecs[i, 0], r_vecs[i, 1], r_vecs[i, 2],   # Start point
              updates[i, 0], updates[i, 1], updates[i, 2], # Direction
              color="red", alpha= 0.6, arrow_length_ratio=0.1 ,length=np.linalg.norm(updates[i]), normalize=True)

ax.set_xlabel('Rotation X')
ax.set_ylabel('Rotation Y')
ax.set_zlabel('Rotation Z')
ax.set_title('visibility jacobian for all possible rotations\n with camera located at specific twc')

plt.show()