import os
import argparse
from kitti_odometry import KittiEvalOdom

def main():
    dataset_gt_path = "Ground_truth/EuRoC_left_cam/V101_GT.txt"
    results_path ="Ground_truth/EuRoC_left_cam/V101_GT.txt"
    eval_tool = KittiEvalOdom()
     #choices=['scale', '6dof', 'scale_7dof', '7dof'], default='7dof')
    try:
        _, _, ate, rpe_t, rpe_r = eval_tool.eval(
            # test to evaluate KITTI odometry ground truth vs ground truth computed from KITTI raw data
            # 'data/kitti_odom/cam_00_gt',
            # os.path.join(args_dict['eval_out_dir'], dataset_name, 'cam_02_est'), # copied from dataset_gt_path
            dataset_gt_path,
            results_path,
            alignment='7dof',
            seqs=[0]
        )
    except ValueError as err:
        print(err)


if __name__ == '__main__':
    main()