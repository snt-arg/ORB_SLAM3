import os
import argparse
from kitti_odometry import KittiEvalOdom
import associate
import numpy as np

def main():
    gt_file = "Ground_truth/EuRoC_left_cam/V101_GT.txt"
    # est_file = "../Results/kf_V101-monotoa_est.txt"
    est_file = "../Results_baseline/kf_V101_monoeuroc.txt"  
    first_list = associate.read_file_list(gt_file, False)
    second_list = associate.read_file_list(est_file , False)
    offset = 0
    max_difference = 20000000
    scale = 1
    matches = associate.associate(first_list, second_list,offset,max_difference)  
    print(len(matches))  
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    first_list_data = np.matrix([[float(value) for value in first_list[a][0:7]] for a,b in matches])
    second_list_data = np.matrix([[float(value)*scale for value in second_list[b][0:7]] for a,b in matches])
    eval_tool = KittiEvalOdom()
     #choices=['scale', '6dof', 'scale_7dof', '7dof'], default='7dof')
    try:
        _, _, ate, rpe_t, rpe_r = eval_tool.eval(
            # test to evaluate KITTI odometry ground truth vs ground truth computed from KITTI raw data
            # 'data/kitti_odom/cam_00_gt',
            # os.path.join(args_dict['eval_out_dir'], dataset_name, 'cam_02_est'), # copied from gt_file
            first_list_data,
            second_list_data,
            # alignment='7dof',
            alignment='7dof',
            seqs=[0]
        )
    except ValueError as err:
        print(err)


if __name__ == '__main__':
    main()