#!/bin/bash

if [ "$#" -lt 4 ]; then
    echo "Usage: $0 <base_dir> <reg_dir> <reg_sub_sampling_interval> <pytorch_env_name>"
    exit 1
fi


base_dir="$1"
reg_dir="$2"
reg_sub_sampling_interval="$3"
pytorch_env_name="$4"

python read_bag.py "$reg_dir" "$reg_sub_sampling_interval" 

eval "$(conda shell.bash hook)"
conda activate "$pytorch_env_name"

./registration.sh "$base_dir" "$reg_dir"

./calculate_registration_error.sh "$base_dir" "$reg_dir"

python eval_pose_error.py "$base_dir"/pose_errors.txt
mv "error_stat.txt" "$reg_dir"/
mv "$base_dir"/"pose_errors.txt" "$reg_dir"/

 ./visualize.sh "$base_dir" "$reg_dir"
 mv "$base_dir"/viz "$reg_dir"/

