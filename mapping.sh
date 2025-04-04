#!/bin/bash

if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <base_dir> <subsampling_interval> <pytorch conda activate env name >" 
    exit 1
fi


# Assign arguments to variables
base_dir="$1"

subsampling_interval="$2"

pytorch_env_name="$3"

python read_bag.py "$base_dir" "$subsampling_interval"

eval "$(conda shell.bash hook)"
conda activate "$pytorch_env_name"

./reconstruct.sh "$base_dir"
./extract_3dpoints_and_view_dir.sh "$base_dir"


