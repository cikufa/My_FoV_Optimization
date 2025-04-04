#!/bin/bash

if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <base_dir> <reg_dir> <reg_sub_sampling_interval>"
    exit 1
fi


base_dir="$1"
reg_dir="$2"
reg_sub_sampling_interval="$3"

python read_bag.py "$reg_dir" "$reg_sub_sampling_interval" 

./Manifold_cpp/build/manifold_test_trajectory "$base_dir" "$reg_dir"

python analyze_point_uncertainty.py "$base_dir"
