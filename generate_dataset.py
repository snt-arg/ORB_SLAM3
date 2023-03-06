import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation



TWB = np.array([[0.33638, -0.01749, 0.94156, 0.06901],
                [-0.02078, -0.99972, -0.01114, -0.02781],
                [0.94150, -0.01582, -0.33665, -0.12395],
                [0.0, 0.0, 0.0, 1.0]])

TCB = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                [0.0, 0.0, 0.0, 1.0]])  

TWC = np.linalg.inv(TCB)@TWB                             


# VERY IMPORTANT NOTE: Landmarks are in camera frame NOT worldframe
landmarks = np.array([
    [5, 6, 3],  # Landmark 1
    [-6, 15, 5],  # Landmark 2
    [10, -3, 8],  # Landmark 3
])
# Define the function to calculate the distance from a pose to the landmarks
def calculate_distances(T, pose):
    distances = []
    for landmark in landmarks:
        landmark_augment = np.append(landmark, 1)
        lm = T@landmark_augment
        distance = np.linalg.norm(pose - lm[:-1])
        distances.append(distance)
    return distances

# Open the CSV file
with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/vicon0/data.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    

    # Open the output file
    with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/toa/data.csv', 'w') as txtfile:

        # Loop over each row in the CSV file
        for i, row in enumerate(csvreader):
            if i == 0:
                initpose = np.array([float(x) for x in row[1:]])
                q = initpose[3:7]
                t = initpose[0:3]
                # Convert the quaternion to a 3x3 rotation matrix
                r = Rotation.from_quat(q).as_matrix()
                # Create a 4x4 transformation matrix by combining the rotation matrix and translation vector
                T2 = np.vstack([np.hstack([r, t[:, np.newaxis]]), np.array([0, 0, 0, 1])])
                # print("This is T2 \n", T2)
                # print("this is t \n ", t)
                initipose_mat = TWC@ T2
                # print("This is TWC \n", TWC)
                # Extract the quaternion and translation from the new pose
                q_new = Rotation.from_matrix(initipose_mat[:3, :3]).as_quat()
                t_new = initipose_mat[:3, 3]
                pose_init_camera = np.hstack([t_new, q_new])
                print("This is the inital pose of the camera\n", pose_init_camera)
                print("This is the Transfromation matrix from world to camera \n", initipose_mat)
                print("This is the T^1 \n", np.linalg.inv(initipose_mat))
            # Parse the pose data from the row
            timestamp = row[0]

            pose_w = np.array([float(x) for x in row[1:4]])
            # pose_w_aug = np.append(pose_w, 1)
            # pose_c_aug = TWC@TWB@pose_w_aug
            # # pose_c_aug = TCB@pose_w_aug
            # pose_c = pose_c_aug[:3]






            # Calculate the distances from the pose to the landmarks
            distances = calculate_distances(np.linalg.inv(initipose_mat),pose_w)

            # Write the timestamp and distances to the output file
            txtfile.write('{},{}\n'.format(timestamp, ','.join(str(x) for x in distances)))


            # txtfile.write('{} {}\n'.format(timestamp, ' '.join(str(x) for x in pose_c)))

for landmark in landmarks:
    landmark_augment = np.append(landmark, 1)
    lm = np.linalg.inv(initipose_mat)@landmark_augment
    print("This is the landmark with constant transfromation \n", lm[:-1])





