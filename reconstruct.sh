#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <base_dir>"
    exit 1
fi

# Assign arguments to variables
base_dir="$1"

cd "$base_dir"
python ~/colmap_utils/reconstruct_from_known_poses.py ./ --img_to_colmap_cam_list ./img_nm_to_colmap_cam.txt --img_to_colmap_pose_list ./img_name_to_colmap_Tcw.txt  --overwrite_db