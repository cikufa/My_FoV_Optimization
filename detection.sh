#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <base_dir> <subsampling_interval>" 
    exit 1
fi


# Assign arguments to variables
base_dir="$1"

subsampling_interval="$2"

python read_bag.py "$base_dir" "$subsampling_interval"

python /media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/cv2_detect.py "$base_dir"
