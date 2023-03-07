#!/bin/bash
pathDatasetEuroc='Datasets/EuRoC' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/V101 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/V101.txt V101_inertial
