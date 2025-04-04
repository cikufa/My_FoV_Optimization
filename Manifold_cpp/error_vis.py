import numpy as np
import matplotlib.pyplot as plt

# Load accuracy data from accfile (CSV format assumed)
acc_data = np.loadtxt("../Data/0_1_optimizer_accuracy_file.csv",delimiter=",")  # Columns: [degree_between, brute_force_max_visibility, optimized_max_visibility, visibility_between]

degree_between = acc_data[:, 0]
brute_force_vis = acc_data[:, 1]
optimized_vis = acc_data[:, 2]
visibility_between = acc_data[:, 3]

# Scatter plot of brute force vs. optimized visibility
plt.figure(figsize=(8, 6))
plt.scatter(brute_force_vis, optimized_vis, c=degree_between, cmap="coolwarm", edgecolors="black", alpha=0.7)

# Add reference line (y=x) for perfect match
plt.plot([min(brute_force_vis), max(brute_force_vis)], [min(brute_force_vis), max(brute_force_vis)], 'k--', label="Perfect Match")

plt.colorbar(label="Degree Difference")
plt.xlabel("Brute Force Max Visibility")
plt.ylabel("Optimized Max Visibility")
plt.title("Brute Force vs. Optimized Visibility")
plt.legend()
plt.grid()
plt.show()

# Calculate absolute difference between brute force and optimized visibility
visibility_diff = np.abs(brute_force_vis - optimized_vis)

plt.figure(figsize=(10, 5))
plt.bar(range(len(visibility_diff)), visibility_diff, color="orange", alpha=0.7)

plt.xlabel("Sample Index")
plt.ylabel("Visibility Difference")
plt.title("Difference Between Brute Force and Optimized Visibility")
plt.grid(axis='y', linestyle="--")
plt.show()
#_________________________________________________________________________________________________________________________--


# import matplotlib.pyplot as plt
# import numpy as np
# from matplotlib import rc


# which_twc= 7
# _save_ext = ".png"

# rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
# rc('text', usetex=True)
# error_data = np.loadtxt("../Data/errorcurve.csv", delimiter=",")
# n= len(error_data)/16380 #number of direction samples.
# error = np.array_split(error_data, n) #(16380, 1)

# directions = np.loadtxt("../brute_force_xyz_indexes_two_degree.csv", delimiter=",") #(16380,3)
# fig = plt.figure()
# ax= fig.add_subplot(111, projection='3d')
# ax.scatter(directions[:, 0], directions[:, 1], directions[:, 2], 
#                 c=error[which_twc], alpha=0.3, cmap='viridis')
# # ____________________________________________

# with open("../Data/Rwhileopt.csv", "r") as f:
#     lines = f.read().strip().split("\n\n")
# Rs = [np.loadtxt(line.splitlines()) for line in lines]
# Rs= Rs[which_twc]
# N=Rs.shape[0] // 3
# r_vecs= Rs.reshape(N, 3)  
# ax.scatter(r_vecs[:, 0], r_vecs[:, 1], r_vecs[:, 2], c=np.arange(len(r_vecs)), cmap="viridis", s=50, label="Optimization Steps")

# updates = np.diff(r_vecs, axis=0) 
# for i in range(len(updates)):
#     ax.quiver(r_vecs[i, 0], r_vecs[i, 1], r_vecs[i, 2],   # Start point
#               updates[i, 0], updates[i, 1], updates[i, 2], # Direction
#               color="red", alpha= 0.5, arrow_length_ratio=0.1 ,length=np.linalg.norm(updates[i]), normalize=True)
    


# ax.set_xlabel('Rotation X')
# ax.set_ylabel('Rotation Y')
# ax.set_zlabel('Rotation Z') 
# ax.set_title('visibility error for all possible directions\n with camera located at specific twc')
# plt.show()

