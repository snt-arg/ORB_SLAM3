#!/bin/bash



ROOT_DIR="/home/meisam/ORB_SLAM3"
DATA_FILE="$ROOT_DIR/Datasets/EuRoC/V101/mav0/vicon0/data.csv"
OUTPUT_FILE1="$ROOT_DIR/results_tum_format/inertial/f_V101_imu_tum.txt" 
OUTPUT_FILE2="$ROOT_DIR/results_tum_format/inertial/f_V101_inertial_tum.txt" 


# convert all files
# python evaluation/convert_tum_format.py
# Run the command with the variables
# conda activate gts_env

echo "Help for using evo_ape using Euroc dataset"
evo_ape euroc -h
echo "plotting the trajectory of the ground truth"
python evaluation/convert_tum_format.py

# echo "Evaluation for ORB-SLAM3 with TOA included"
# evo_traj tum OUTPUT_FILE1 -p --plot_mode xy 

echo "Evaluation for ORB-SLAM3 with TOA included"
evo_ape euroc $DATA_FILE $OUTPUT_FILE1 -va --plot --plot_mode xy --save_results "$ROOT_DIR/results/ORB.zip" --t_max_diff 0.005 --plot_full_ref    

echo "Evaluation for ORB-SLAM3 with just monocular images"
evo_ape euroc $DATA_FILE $OUTPUT_FILE2 -va --plot --plot_mode xy --save_results "$ROOT_DIR/results/ORB.zip" --t_max_diff 0.005 --plot_full_ref    