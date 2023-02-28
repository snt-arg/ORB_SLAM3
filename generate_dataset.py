import os
import csv
import numpy as np

landmarks = np.array([
    [5, 6, 3],  # Landmark 1
    [-6, 15, 5],  # Landmark 2
    [10, -3, 8],  # Landmark 3
])
# Define the function to calculate the distance from a pose to the landmarks
def calculate_distances(pose):
    distances = []
    for landmark in landmarks:
        distance = np.linalg.norm(pose - landmark)
        distances.append(distance)
    return distances

# Open the CSV file
with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/vicon0/data.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    

    # Open the output file
    with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/toa_gt.csv', 'w') as txtfile:

        # Loop over each row in the CSV file
        for row in csvreader:

            # Parse the pose data from the row
            timestamp = row[0]
            pose = np.array([float(x) for x in row[1:4]])

            # Calculate the distances from the pose to the landmarks
            distances = calculate_distances(pose)

            # Write the timestamp and distances to the output file
            txtfile.write('{} {}\n'.format(timestamp, ' '.join(str(x) for x in distances)))





