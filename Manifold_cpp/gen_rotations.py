import numpy as np
import csv

def euler_to_rotation_matrix(alpha, beta, gamma):
    """
    Convert Euler angles (ZYZ convention) to a 3x3 rotation matrix.
    - alpha: Z-axis rotation (degrees)
    - beta: Y-axis rotation (degrees)
    - gamma: Z-axis rotation (degrees)
    """
    alpha = np.radians(alpha)
    beta = np.radians(beta)
    gamma = np.radians(gamma)
    
    # Individual rotation matrices
    Rz_alpha = np.array([
        [np.cos(alpha), -np.sin(alpha), 0],
        [np.sin(alpha),  np.cos(alpha), 0],
        [0, 0, 1]
    ])
    
    Ry_beta = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])
    
    Rz_gamma = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma),  np.cos(gamma), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = Rz_alpha @ Ry_beta @ Rz_gamma
    return R

# Generate all rotations with 2-degree resolution and save to file
with open("../all_rotation.csv", 'w', newline='') as f:
    writer = csv.writer(f)
    # Iterate through all Euler angle combinations:
    for alpha in range(0, 360, 2):       # Z-axis (0° to 358°)
        for beta in range(0, 181, 2):    # Y-axis (0° to 180°)
            for gamma in range(0, 360, 2): # Z-axis (0° to 358°)
                R = euler_to_rotation_matrix(alpha, beta, gamma)
                # Flatten matrix into a single row of 9 columns
                # line = ",".join([f"{x:.6f}" for x in R.flatten()]) + "\n"
                writer.writerow(R.flatten())

print("Rotation matrices saved to 'rotation_matrices.csv'")
