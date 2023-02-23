#!/bin/bash
pathDatasetEuroc='Datasets/EuRoC' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
# echo "Launching MH01 with Monocular-Inertial sensor"
# ./Examples/Monocular-Inertial/mono_inertial_toa_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi


#------------------------------------
# Monocular Examples
echo "Launching VH01"
./Examples/Monocular-Inertial/mono_inertial_toa_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Examples/Monocular/EuRoC_TimeStamps/V101.txt dataset-V101_monoMonocular
