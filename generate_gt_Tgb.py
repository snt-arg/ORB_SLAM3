import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation
import sys
sys.path.append('./evaluation')
import associate
import subprocess


# This code assumes orbslam results are in camera 0 frame

#The pose of Vicon wrt body frame
Tbv = np.array([[0.33638, -0.01749, 0.94156, 0.06901],
                [-0.02078, -0.99972, -0.01114, -0.02781],
                [0.94150, -0.01582, -0.33665, -0.12395],
                [0.0, 0.0, 0.0, 1.0]])
#The pose of the Camera wrt body frame
Tbc = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                [0.0, 0.0, 0.0, 1.0]])  




Tumeyama = np.array([[ 0.36852209,  0.24800063,  0.8959281,  0.83038384 ],
 [ 0.09182906, -0.9687563,   0.23038806, 2.19203329],
 [ 0.92507237, -0.00263085, -0.37978177, 1.05654092],
 [0, 0,0 , 1]])

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


A = np.identity(n=4)
q = Rotation.from_matrix(A[:3, :3]).as_quat()
print(q)

# Open the CSV file
with open('/home/meisam/ORB_SLAM3/Datasets/EuRoC/V101/mav0/vicon0/data.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    

    # Open the output file
    with open('/home/meisam/ORB_SLAM3/Gt_V101_gb.txt', 'w') as txtfile:

        # Loop over each row in the CSV file
        for i, row in enumerate(csvreader):

            timestamp = row[0]
            vpose = np.array([float(x) for x in row[1:]])
            pose_gb = get_transformation_matrix(vpose)@np.linalg.inv(Tbv)
            vPose_gb = get_pose_from_transformation_matrix(pose_gb)
            # Write the timestamp and distances to the output file
            txtfile.write('{},{}\n'.format(timestamp, ','.join(str(x) for x in vPose_gb)))



# offset = 0.0
# max_difference = 20000000
# first_file = "Gt_V101_gb.txt"
# second_file = "Results_baseline/euroc_inertial/f_V101_inertial.txt"
# first_list = associate.read_file_list("Gt_V101_gb.txt", False)
# second_list = associate.read_file_list("Results_baseline/euroc_inertial/f_V101_inertial.txt", False)

# matches = associate.associate(first_list, second_list,offset,max_difference)   

# result = subprocess.run(['python', 'evaluation/associate.py', 'Gt_V101_gb.txt', 'Results_baseline/euroc_inertial/f_V101_inertial.txt', '--offset', '0', '--max_difference', '20000000' ], capture_output=True, text=True)
# python evaluation/associate.py Gt_V101_gb.txt Results_baseline/euroc_inertial/f_V101_inertial.txt  --offset  0 --max_difference  20000000





