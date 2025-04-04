#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <base_dir> <reg_sub_sampling_interval>"
    exit 1
fi


base_dir="$1"
reg_sub_sampling_interval="$2"
echo "./$base_dir/sparse/0/"

mkdir -p "./$base_dir/sparse/0/"

cp "./$base_dir/segmented_pointcloud.txt" "./$base_dir/sparse/0/stripped_points3D.txt"

./Manifold_cpp/build/manifold_test_trajectory "$base_dir" "$base_dir"

python analyze_point_uncertainty.py "$base_dir"
