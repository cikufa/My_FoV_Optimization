#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <base_dir> [reg_dir]"
    exit 1
fi

# Assign arguments to variables
base_dir="$1"
reg_dir="$2"

cd ./"$reg_dir"
python ~/colmap_utils/generate_img_rel_path.py --base_dir ../"$base_dir"/images --img_dir ./images --output_dir ./images --img_nm_to_cam_list ./img_nm_to_colmap_cam.txt

cd ../"$base_dir"
python ~/colmap_utils/register_images_to_model.py ./ --reg_name test --upref_no_time --reg_list_fn ../"$reg_dir"/images/rel_img_path.txt --img_nm_to_colmap_cam_list ../"$reg_dir"/images/rel_img_nm_to_cam_list.txt