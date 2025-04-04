import numpy as np
import csv

class BruteForce:
    def __init__(self, angular_resolution_deg, filename):
        self.directions = self.generate_directions(angular_resolution_deg)
        self.save_directions_to_csv(filename)
        print(f"Directions saved to CSV file: {filename}")

    def generate_directions(self, angular_resolution_deg):
        directions = []
        angular_resolution_rad = np.radians(angular_resolution_deg)

        # Sample azimuth angle from 0 to 360 degrees
        for azimuth in np.arange(0, 360, angular_resolution_deg):
            # Sample elevation angle from -90 to 90 degrees
            for elevation in np.arange(-90, 91, angular_resolution_deg):
                # Convert spherical coordinates (azimuth, elevation) to Cartesian coordinates
                azimuth_rad = np.radians(azimuth)
                elevation_rad = np.radians(elevation)

                # Calculate direction vector
                x = np.cos(elevation_rad) * np.cos(azimuth_rad)
                y = np.cos(elevation_rad) * np.sin(azimuth_rad)
                z = np.sin(elevation_rad)

                # Normalize the direction vector
                direction = np.array([x, y, z])
                direction /= np.linalg.norm(direction)
                directions.append(direction)
                
    
        return np.array(directions)

    def save_directions_to_csv(self, filename):
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            for direction in self.directions:
                writer.writerow(direction)

if __name__ == "__main__":
    # Generate and save directions with 2-degree angular resolution
    BruteForce(2.0, "brute_force_xyz_indexes_two_degree_nonorm.csv")




# import numpy as np
# import csv

# class BruteForce:
#     def __init__(self, angular_resolution_deg, filename):
#         self.directions, self.angles = self.generate_directions(angular_resolution_deg)
#         self.save_directions_to_csv(filename)
#         print(f"Directions saved to CSV file: {filename}")

#     def generate_directions(self, angular_resolution_deg):
#         directions = []
#         angles = []  # Store angles in degrees
#         angular_resolution_rad = np.radians(angular_resolution_deg)

#         # Sample azimuth (yaw) from 0 to 360 degrees
#         for azimuth in np.arange(0, 360, angular_resolution_deg):
#             # Sample elevation (pitch) from -90 to 90 degrees
#             for elevation in np.arange(-90, 91, angular_resolution_deg):
#                 # Convert to radians
#                 azimuth_rad = np.radians(azimuth)
#                 elevation_rad = np.radians(elevation)

#                 # Compute direction vector
#                 x = np.cos(elevation_rad) * np.cos(azimuth_rad)
#                 y = np.cos(elevation_rad) * np.sin(azimuth_rad)
#                 z = np.sin(elevation_rad)

#                 # Normalize
#                 direction = np.array([x, y, z])
#                 direction /= np.linalg.norm(direction)
                
#                 # Store direction and angles
#                 directions.append(direction)
#                 angles.append([azimuth, elevation, 0])  # Roll = 0 (fixed)

#         return np.array(directions), np.array(angles)

#     def save_directions_to_csv(self, filename):
#         with open(filename, 'w', newline='') as file:
#             writer = csv.writer(file)
#             for direction, angle in zip(self.directions, self.angles):
#                 writer.writerow(list(direction) + list(angle))

# if __name__ == "__main__":
#     BruteForce(2.0, "brute_force_xyz_angles.csv")  # Saves directions & angles





