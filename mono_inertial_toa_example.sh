#!/bin/bash
pathDatasetEuroc='Datasets/EuRoC' #Example, it is necesary to change it by the dataset path

#------------------------------------
# IMU-Monocular Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./Examples/Monocular-Inertial/mono_inertial_toa_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/V101.txt V101_imu


#------------------------------------
# # Monocular Examples
# echo "Launching VH01"
# ./Examples/Monocular-Inertial/mono_inertial_toa_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Examples/Monocular/EuRoC_TimeStamps/V101.txt V101-Inertial
