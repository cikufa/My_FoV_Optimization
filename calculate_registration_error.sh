#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <base_dir> <reg_dir>"
    exit 1
fi
base_dir="$1"
reg_dir="$2"
cd ./"$base_dir"
python ~/colmap_utils/calculate_pose_errors.py --reg_model_dir ./test_sparse  --reg_img_name_to_colmap_Tcw ../"$reg_dir"/img_name_to_colmap_Tcw.txt --reg_img_dir ../"$reg_dir"/images  --output_path ./