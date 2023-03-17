import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

# This code assumes orbslam results are in camera 0 frame

#The pose of Vicon wrt body frame
TBV = np.array([[0.33638, -0.01749, 0.94156, 0.06901],
                [-0.02078, -0.99972, -0.01114, -0.02781],
                [0.94150, -0.01582, -0.33665, -0.12395],
                [0.0, 0.0, 0.0, 1.0]])
#The pose of the Camera wrt body frame
TBC = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                [0.0, 0.0, 0.0, 1.0]])

# What we want is T camera wrt to the world frame TCW (world of Euroc),  for that first we need TVC
# TVC = TVB @ np.linalg.inv(TBC)
# TVC = TVB*TBC
TVC = np.linalg.inv(TBV)@TBC



Tumeyama = np.array([[ 0.36852209,  0.24800063,  0.8959281,  0.83038384 ],
 [ 0.09182906, -0.9687563,   0.23038806, 2.19203329],
 [ 0.92507237, -0.00263085, -0.37978177, 1.05654092],
 [0, 0,0 , 1]])

# ground truth is vicon wrt the world:  gt_wv = pwv
# So if we want to obtain the camera pose wrt the world frame of euroc
# GT_wc = GT_wv * TVB*TBC or gt_wc = GT_wv * TVC
# landmarks are also in the world frame and because there is nothing to do with the attitude, we dont need ay transforamtion

# So now we now by TVC we can convert every ground truth pose to the pose of the camera in the world frame of euroc
# the first ground truth of the camera GT_WV0*TVC is the origin for the ORBSLAM3
# Let just say GTWV0 is GT0, the first row in the vicon data set
# SO T_world_ORB = GT0 => T_ORB_world = np.linalg.inv(GT0)
# Now we want everything in the ORB frame
# 1) Ground truths:
    # GT_ORB_camera = T_ORB_world* GT_wc =  T_ORB_world* GT_wv * TVC
# 2) Landmarks:
    # LM_ORB = T_ORB_world* LM_world




# VERY IMPORTANT NOTE: Landmarks in world frame (euroc world frame)
landmarks = np.array([
    [5, 6, 3],  # Landmark 1
    [-6, 15, 5],  # Landmark 2
    [10, -3, 8],  # Landmark 3
])
# define function that takes the (x,y,z, qw,qx,qy,qz) and returns the transformation matrix
def get_transformation_matrix(pose):
    q = pose[3:7]
    q_new = np.concatenate((q[1:], q[0].reshape(1,)))
    t = pose[0:3]
    # Convert the quaternion to a 3x3 rotation matrix
    R = Rotation.from_quat(q_new ).as_matrix()
    # Create a 4x4 transformation matrix by combining the rotation matrix and translation vector
    T = np.vstack([np.hstack([R, t[:, np.newaxis]]), np.array([0, 0, 0, 1])])
    return T
# define a function that takes the transformation matrix and returns the (x,y,z, qw,qx,qy,qz)
def get_pose_from_transformation_matrix(T):
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    t = T[:3, 3]
    pose = np.hstack([t, q])
    return pose





# Define the function to calculate the distance from a pose to the landmarks
# This function takes the GT (vicon wrt the world, euroc dataset) and landmark wrt to the world,
# # then calculate distance of the camera wrt to the orb origin which is the first ground truth
# def calculate_distances(T_ORB_world, pose):
#     distances = []
#     for landmark in landmarks:
#         landmark_augment = np.append(landmark, 1)
#         # calculate the pose of landmarks wrt to the ORB frame (the first ground truth)
#         lm = T_ORB_world@landmark_augment
#         pose_aug = np.append(pose, 1)
#         # calculate the pose of the camera wrt to the ORB frame (the first ground truth)
#         pose_aug_transformed = T_ORB_world@pose_aug@TVC
#         distance = np.linalg.norm(pose_aug_transformed[:-1] - lm[:-1])
#         distances.append(distance)
#     return distances

# Open the CSV file
with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/vicon0/data.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)


    # Open the output file
    with open('/home/meisam/Desktop/test_python/EuRoCV101_transfomred.txt', 'w') as txtfile:

        # Loop over each row in the CSV file
        for i, row in enumerate(csvreader):
            if i == 0:
                initpose = np.array([float(x) for x in row[1:]])
                q = initpose[3:7]
                q_new = np.concatenate((q[1:], q[0].reshape(1,)))
                print (q_new)
                t = initpose[0:3]
                # Convert the quaternion to a 3x3 rotation matrix
                R_world_V0 = Rotation.from_quat(q_new).as_matrix()
                # Create a 4x4 transformation matrix by combining the rotation matrix and translation vector
                T_world_V0 = np.vstack([np.hstack([R_world_V0, t[:, np.newaxis]]), np.array([0, 0, 0, 1])])
                # T_world_C0 = T_world_V0@TVC
                T_world_B0 = T_world_V0@np.linalg.inv(TBV)
                T_ORB_world = np.linalg.inv(T_world_B0)

                print("This is the T_ORB_world \n", T_ORB_world)
                print("This is the dfsdfsdfsf  \n", T_ORB_world@T_world_V0)

            timestamp = row[0]
            pose_wv = np.array([float(x) for x in row[1:]])
            GT_wv =get_transformation_matrix(pose_wv)
            # GT_orb_w = T_ORB_world@GT_wv@TVC
            GT_orb_B = T_ORB_world@GT_wv@np.linalg.inv(TBV)
            pose_final = get_pose_from_transformation_matrix(GT_orb_B)


            # Write the timestamp and distances to the output file
            txtfile.write('{},{}\n'.format(timestamp, ','.join(str(x) for x in pose_final)))


            # txtfile.write('{} {}\n'.format(timestamp, ' '.join(str(x) for x in pose_c)))

# for landmark in landmarks:
#     landmark_augment = np.append(landmark, 1)
#     lm = T_ORB_world@landmark_augment
#     print("This is the landmark with constant transfromation \n", lm[:-1])
# print(T_ORB_world)



# load the data from the text file
data = np.loadtxt('/home/meisam/Desktop/test_python/EuRoCV101_transfomred.txt', delimiter=',')
data2 = np.loadtxt('/home/meisam/ORB_SLAM3/Results/f_V101_imu.txt', delimiter=' ')

# extract the second and third columns
col2 = data[:, 1]
col3 = data[:, 2]

# extract the second and third columns
col2_est = data2[:, 1]
col3_est = data2[:, 2]

# plot the data
plt.plot(col2, col3, 'o', markersize=1, color='blue')
plt.plot(col2_est, col3_est, 'o', markersize=3, color='red')
plt.xlabel('Column 2')
plt.ylabel('Column 3')
plt.title('Plot of Column 2 vs. Column 3')
plt.show()