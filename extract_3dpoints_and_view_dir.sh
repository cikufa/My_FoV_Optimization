#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <base_dir>"
    exit 1
fi

# Assign arguments to variables
base_dir="$1"

cd "$base_dir"

python ~/colmap_utils/calculate_point_view_direction.py ./sparse/0/
python ~/colmap_utils/strip_points3d.py ./sparse/0/points3D.txt 