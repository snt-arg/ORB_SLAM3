#!/bin/bash



ROOT_DIR="/home/meisam/ORB_SLAM3"
DATA_FILE="$ROOT_DIR/Datasets/EuRoC/V101/mav0/vicon0/data.csv"
OUTPUT_FILE1="$ROOT_DIR/results_tum_format/f_V101_monotoa_tum.txt" 
OUTPUT_FILE2="$ROOT_DIR/results_tum_format/f_V101_monoeuroc_base_tum.txt" 


# convert all files
# python evaluation/convert_tum_format.py
# evo_ape euroc ../Datasets/EuRoC/V101/mav0/vicon0/data.csv ../results_tum_format/f_V101_monotoa_tum.txt -va --plot --plot_mode xy --save_results results/ORB.zip --t_max_diff 0.005 --plot_full_ref -s
# Run the command with the variables
evo_ape euroc $DATA_FILE $OUTPUT_FILE1 -va --plot --plot_mode xy --save_results "$ROOT_DIR/results/ORB.zip" --t_max_diff 0.005 --plot_full_ref -s

evo_ape euroc $DATA_FILE $OUTPUT_FILE2 -va --plot --plot_mode xy --save_results "$ROOT_DIR/results/ORB.zip" --t_max_diff 0.005 --plot_full_ref -s