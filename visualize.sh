#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <base_dir> <reg_dir>"
    exit 1
fi
base_dir="$1"
reg_dir="$2"

cd "$base_dir"
directory="./viz"

if [ ! -d "$directory" ]; then
    mkdir "$directory"
else
    echo "Directory already exists: $directory"
fi

python ~/colmap_utils/viz_matched_landmarks.py --database ./test_database.db --img_dir ./images --model_dir ./test_sparse --rel_img_list ../"$reg_dir"/images/rel_img_path.txt  --outdir ./viz --make_video