# FOV_Optimization_On_Manifold
This repository contains the code and simulation results for the research project titled "Maximizing Feature Visibility by Online On-Manifold Optimization For Motion-Decoupled Cameras"

## Abstract
Robotic perception algorithms often assume that a sensor's field-of-view (FoV) is fixed with respect to the robot. This assumption limits the flexibility of robot planning and perception systems. However, with the advent of motion-decoupled sensors, such as PTZ cameras, gimbals, and RF radar systems, it is now possible to separate the orientation of the sensor from the robot's movements, enabling better perception and planning capabilities.

This project proposes a novel on-manifold optimization technique for real-time sensor view generation. By optimizing the sensor's rotation to maximize the number of visible features within its limited FoV, the system can be applied to various perception tasks such as localization, mapping, and tracking. This method can either maximize visual features or metrics like Fisher Information for enhanced perception.

![Demonstration of the Algorithm](https://github.com/droneslab/FOV-Optimization-on-Manifold/blob/main/Fov%20demo.gif)

![subsampled_setup2](https://github.com/user-attachments/assets/b8100779-c6ff-46ad-96bc-4912e08ecea5)

## Evaluation
### Simulation Results

The evaluation of the proposed On-Manifold Optimization method for maximizing feature visibility in motion-decoupled sensors was conducted in both simulated environments and real-world experiments.
Toy Example:
        A smaller-scale example was run with 100 feature points to demonstrate the effectiveness of the proposed FoV optimization. Results show convergence towards the mean center of all feature points without any field-of-view (FoV) limits and accurate feature cluster detection when a 45-degree FoV limit is imposed.
        This small-scale experiment validates that the optimization algorithm accurately finds the cluster of features within the constrained sensor's FoV.

Monte-Carlo Grid Optimization:
        The optimization method was further tested on a larger grid with 15,000 features, sub-sampled in intervals of 2,500 features. The evaluation grid had a resolution of 20x20x2.
        The results show that our method achieves similar performance to brute-force search with a fraction of the computation time. Our approach processes each grid location in 100–650ms, while the brute-force search takes up to 90 seconds.
        Overall, our method performed well against brute-force search, detecting 87.7% of the features while being real-time capable.

![combined_image](https://github.com/user-attachments/assets/fe572442-2602-48dc-b9fe-ff2481d4ed05)

## Real-World Experiments

  Robot Platform: The system was tested on the Boston Dynamics Spot quadruped robot equipped with a motion-decoupled camera and Realsense D455 sensor.
    Test Environment: We evaluated the camera’s ability to track and register features in an indoor environment with a pre-planned S-shape trajectory.
    Registration Success Rate:
        The optimized sensor trajectory achieved a feature registration success rate of 99.04% compared to 70.48% for the unoptimized trajectory.
        This significant improvement demonstrates the ability of our algorithm to actively orient the sensor to capture more features within its limited FoV, thereby improving perception in real-world scenarios.
        ![subsampled_setup](https://github.com/user-attachments/assets/1ee122a1-d71a-4cee-b9de-5d01c7916c51)


## Computation Time and Resource Usage

Computation Time: The optimization algorithm showed significant improvements in terms of time efficiency compared to previous brute-force and map-based methods.
        For a grid of 15,000 features, our method takes around 460ms, while brute-force approaches take up to 90 seconds.
        In dynamic scenarios, such as real-time robot navigation, this improvement in computation time allows for real-time feature maximization, making the method suitable for time-sensitive applications.
    Resource Usage: Due to the on-manifold optimization approach, the algorithm operates efficiently without requiring pre-built feature maps, which are resource-intensive and computationally expensive to update.

## Visual Localization Accuracy

The proposed optimization was integrated into a visual localization system and tested in a photo-realistic simulation environment. Compared to existing methods, our system reduced localization error in three out of six cases while requiring significantly less computational time.
    These results demonstrate that our method is not only faster but also competitive in terms of localization accuracy, making it suitable for applications requiring fast and reliable visual perception.

## Overall Performance

Our proposed method efficiently maximizes feature visibility for motion-decoupled sensors, making it highly suitable for applications in robotic navigation, tracking, and mapping.
    The evaluation demonstrated that the system works effectively in both simulation and real-world environments, providing a significant improvement in perception and planning tasks compared to traditional approaches.
    
## How to use

### Install COLMAP& colmap_utils
https://github.com/uzh-rpg/colmap_utils

### Clone Repo

```
git clone git@github.com:yuyangch/FOV_Optimization_On_Manifold.git
```
### Install Pytorch
https://pytorch.org/get-started/locally/


## Pipeline Overview
The process is split into three stages, Mapping, Planning and Registration stage.

## Monte Carlo Grid Experiment

### Quick Start

Generate a clustered map + sampled poses (and subsamples + manifest):

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/generate_cluster_map.py \
  --name clusters4_demo \
  --features-per-cluster 500,800,1200,1500 \
  --bounds -250 250 -250 250 0 10 \
  --pose-resolution 20,20,1 \
  --seed 42 \
  --subsample-levels 10 \
  --subsample-prefix 0
```

Run Monte Carlo on the base map (auto-detects manifest + pose file):

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --build
```

Run all subsample levels in one run folder:

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --all-levels \
  --build
```

Run Monte Carlo on a specific subsample level:

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --level 3 \
  --build
```

Choose brute-force objective mode:

```bash
# Default: both objectives
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --all-levels \
  --bf-objective both \
  --build

# Feature-count objective only
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --all-levels \
  --bf-objective feat \
  --build

# Visibility objective only
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --all-levels \
  --bf-objective vis \
  --build
```

Use the backup manifold implementation:

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/run_monte_carlo_experiment.py \
  --map-name clusters4_demo \
  --level 3 \
  --manifold backup \
  --build
```

Plot results (charts saved as PNGs, 3D viewer only on `--show`):

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/plot_monte_carlo_results.py \
  --run-dir /home/shekoufeh/fov_ws/My_FoV_Optimization/Results/monte_carlo/<timestamp>_<label>
```

### Interactive Monte Carlo Viewer (per-iteration quivers)

Launch an interactive GUI where you can:
- pick a pose (“cluster location”) from the generated pose CSV
- click **Optimize** to stream per-iteration quivers (red) moving toward the final optimized direction
- click **Brute Force** to show the brute-force quiver (blue) for that pose

Run (auto-detects the `*_map_manifest.yaml` under `Map/` when only one exists):

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/interactive_monte_carlo_viewer.py
```

If you have multiple manifests, point to one explicitly:

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/interactive_monte_carlo_viewer.py \
  --map-manifest /home/shekoufeh/fov_ws/My_FoV_Optimization/Map/<map_folder>/<name>_map_manifest.yaml
```

Optional flags:
- `--build` builds `Manifold_cpp/build/manifold_test` before running.
- `--speed-ms 60` controls animation speed (frame delay).
- `--max-points 6000` caps rendered map points for faster plotting.

Outputs (CSV + plots) are saved under:
`Results/monte_carlo/<timestamp>_<label>/data` (raw CSVs)
`Results/monte_carlo/<timestamp>_<label>/` (plots + analysis)

Pipeline notes:
- The generator writes a manifest next to the map CSV: `<map_csv_stem>_manifest.yaml` (contains `pose_map` and `subsample_maps` when subsampling is enabled).
- The runner auto-detects the pose file from the map name (or manifest) and caches brute-force results in `Results/monte_carlo/<timestamp>_<label>/bf_cache/` (keyed by map+pose+objective).
- For `--all-levels`, per-level cache files are stored in the same run cache folder as `0_1_bf_cache.csv`, `0_2_bf_cache.csv`, ..., `0_10_bf_cache.csv`.
- The plotter reads `run_info.txt` to locate the correct map and log slice for this run.

Performance + cache controls (env vars):
- `FOV_USE_OPENMP=1` enables OpenMP over poses. Optional: `FOV_NUM_THREADS=8`.
- `FOV_NO_IO=1` skips per-iteration logs (faster, fewer debug files). Enabled automatically when `FOV_USE_OPENMP=1`.
- `FOV_BF_COARSE_TO_FINE=1` enables coarse-to-fine BF (still exact if `FOV_BF_REQUIRE_GLOBAL=1`, default).
- `FOV_BF_REQUIRE_GLOBAL=0` skips the full BF scan (faster, but not guaranteed global optimum).
- `FOV_BF_COARSE_STRIDE=5` and `FOV_BF_REFINE_DEG=2` tune coarse stride/refine window.
- `FOV_BF_CACHE_DIR=/path/to/bf_cache` overrides the BF cache location.

Cache safety:
- BF cache files include a signature header. If the signature doesn’t match the current map/pose/grid, the run exits with an error. Delete the cache file or run in a new results folder to recompute.

### Standalone Map Generator

If you want to iterate on clustered maps without running Monte Carlo optimization:

```bash
python /home/shekoufeh/fov_ws/My_FoV_Optimization/scripts/generate_cluster_map.py \
  --clusters 8 \
  --features-per-cluster 1000 \
  --bounds -250 250 -250 250 0 10 \
  --pose-resolution 20,20,1 \
  --seed 42 \
  --subsample-levels 10
```

This saves the base map CSV into `Map/` and a preview image next to it as `<map_csv_stem>_preview.png`.
Subsampled maps are written next to the base map with filename:
`<subsample_prefix>_<level>_<base_map_csv_name>` (for example: `0_1_clusters8_map.csv`).
The generator writes a manifest `<map_csv_stem>_manifest.yaml` next to the base map to keep subsample
level numbering consistent. Use the manifest with `--map-manifest` and select the subsample
level with `--level`, or pass `--map` plus `--level` and the runner will auto-detect the manifest.

### Runner Arguments

Script: `My_FoV_Optimization/scripts/run_monte_carlo_experiment.py`

Inputs:
- `--root`  
  Path to repo root. Default is the parent directory of the script.
- `--build`  
  Build the C++ binaries before running.
- `--cmake`  
  Force re-run `cmake ..` (useful if build files are stale).
- `--manifold`  
  Select which manifold implementation to compile (`current` or `backup`).
- `--grid`  
  Grid resolution as `X,Y,Z` or `XxYxZ`. Example `20x20x2`.  
  This updates the `ExperimentManager(..., 1, X, Y, Z, 1, import_map, ...)` arguments
  in `My_FoV_Optimization/Manifold_cpp/manifold_test.cpp` and triggers a rebuild.
- `--map`  
  Map file to load (relative to `Map/` or absolute). Overrides `--import-map`.
- `--map-manifest`  
  YAML manifest produced by `generate_cluster_map.py`. Use this with `--level`
  to pick a subsample level.
- `--pose-file`  
  Pose CSV file to use (relative to `Map/` or absolute). Overrides manifest pose map.
- `--bounds`  
  Grid bounds `x_low,x_high,y_low,y_high,z_low,z_high`.
- `--import-map`  
  `1` loads map from disk. (`0` is no longer supported.)
- `--clusters`  
  Cluster count (ignored when `--import-map 1`, but required by the binary).
- `--label`  
  Optional label appended to output folder name.
- `--move`  
  Move outputs into the results folder instead of copying.
- `--all-levels`  
  Run all subsample levels from the selected map manifest.
- `--level`  
  Run only one subsample level.
- `--bf-objective`  
  Brute-force objective mode: `both` (default), `feat`, or `vis`.

### Inputs to the C++ experiment

The C++ binary is `My_FoV_Optimization/Manifold_cpp/build/manifold_test`.
It is called as:

```bash
./manifold_test <levels> <import_map> <clusters> <pose_csv>
```

Internals:
- The map path is currently set in `My_FoV_Optimization/Manifold_cpp/manifold_test.cpp`.
- The grid bounds and resolution are also set there.
- The Monte Carlo loop runs in `My_FoV_Optimization/Manifold_cpp/monte_carlo/monte_carlo.h`.
- Downsampling is handled by `My_FoV_Optimization/scripts/generate_cluster_map.py`.

### Outputs

During the run, raw outputs are written to:

`/home/shekoufeh/fov_ws/My_FoV_Optimization/Data`

The runner then collects all files modified in `Data/` and copies or moves them into:

`/home/shekoufeh/fov_ws/My_FoV_Optimization/Results/monte_carlo/<timestamp>_<label>/data`

It also writes:

`/home/shekoufeh/fov_ws/My_FoV_Optimization/Results/monte_carlo/<timestamp>_<label>/run_info.txt`

Common output files (prefix `0_1_`, `0_2_`, ...):
- `*_single_run_rotated_quivers.csv` (optimized quivers)
- `*_single_run_brute_force_rotated_quivers.csv` (brute‑force quivers)
- `*_optimizer_accuracy_file.csv` (generated later by analysis script)
- `*_optimizer_avg_time_file.csv`
- `*_brute_force_avg_time_file.csv`
- `*_bf_cache.csv` (brute‑force cache, in run-specific `bf_cache/`)

BF cache columns:
- `ref_x,ref_y,ref_z`
- `bf_feat_x,bf_feat_y,bf_feat_z`
- `bf_vis_x,bf_vis_y,bf_vis_z`
- `bf_time_us`
- `bf_time_feat_us`
- `bf_time_vis_us`

### Trajectory Optimizer Debug And Timing

For the trajectory optimizer in `My_FoV_Optimization/Manifold_cpp/trajectory_test.cpp`,
the core optimization loop now excludes analysis-only visibility calculations unless you
explicitly re-enable them.

To re-enable the scheduled / post-update / fixed-angle visibility metrics and write them
to a CSV debug log:

```bash
export FOV_OPT_DEBUG_LOG_ENABLED=1
export FOV_OPT_DEBUG_LOG_PATH=/absolute/path/to/optimization_debug.csv
```

You can also use:

```bash
export FOV_OPT_DEBUG_LOG=1
```

If `FOV_OPT_DEBUG_LOG_PATH` is not set, the optimizer writes `optimization_debug.csv`
next to the trajectory outputs.

This debug log includes per-pose, per-iteration analysis values such as:
- scheduled visibility count / score
- post-update visibility count / score
- fixed-angle visibility count / score
- Jacobian components and step statistics

These values are for analysis and debugging. They are not required for the optimizer update
itself, and keeping them disabled gives a cleaner measure of optimization runtime.

The trajectory runner also writes split timing information to:

`<output_dir>/optimization_timing.csv`

with the columns:
- `optimization_core_ms`: pure optimization time after prefiltering, measured with `traj_op.optimize(false)` so debug/eval exports are excluded
- `esdf_prefilter_ms`: time spent in `prefilterVisiblePoints()` when the ESDF backend is used
- `total_measured_ms`: `optimization_core_ms + occlusion_prefilter_ms`
- `depthmap_prefilter_ms`: time spent in `prefilterVisiblePoints()` when the prebuilt visibility depth-map backend is used
- `occlusion_prefilter_ms`: generic prefilter time for whichever occlusion backend actually ran
- `prefilter_backend`: `none`, `esdf`, or `depthmap`

This lets you report optimization time separately from the occlusion prefilter step while
keeping backward-compatible ESDF timing columns. Final trajectory files are still written
after the timer stops.

You can choose the occlusion backend with:

```bash
export FOV_OPT_OCCLUSION_BACKEND=auto
```

Supported values are:
- `auto`: prefer ESDF if loaded, otherwise use the prebuilt visibility depth map if loaded
- `esdf`: force the online ESDF raycast prefilter
- `depthmap`: force the prebuilt visibility depth-map query backend
- `none`: disable occlusion prefiltering

To use the prebuilt visibility depth-map backend, set:

```bash
export FOV_OPT_OCCLUSION_BACKEND=depthmap
export FOV_OPT_VISIBILITY_DEPTHMAP_PATH=/absolute/path/to/depthmap.proto
```

Optional depth-range limits for the depth-map backend:

```bash
export FOV_OPT_VISMAP_MIN_DIST=0.0
export FOV_OPT_VISMAP_MAX_DIST=20.0
```

Aliases `FOV_OPT_DEPTHMAP_PATH`, `FOV_OPT_DEPTHMAP_MIN_DIST`, and
`FOV_OPT_DEPTHMAP_MAX_DIST` are also supported.

When the depth-map backend is used through the batch runner scripts
(`optimize_trajopt.sh` / `run_fov_opt_rrt_none.sh`), the runner now keeps a
single long-lived `manifold_test_trajectory --server` worker alive and loads
`depthmap.proto` once for the whole batch. Each trajectory is then optimized by
sending a request to that worker, which matches the FIF-style "maps already
loaded, planning queried repeatedly" setup more closely.

As a result, per-trajectory `optimization_time_sec.txt` for the depth-map
backend excludes the one-time worker startup and depth-map load, and measures
the per-request runtime instead.

Example runs for each backend:

```bash
# no occlusion prefilter
FOV_OPT_OCCLUSION_BACKEND=none \
/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory \
<input_dir> <output_dir> 1

# online ESDF occlusion prefilter
FOV_OPT_OCCLUSION_BACKEND=esdf \
FOV_OPT_ESDF_PATH=/absolute/path/to/esdf_map.proto \
/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory \
<input_dir> <output_dir> 1

# prebuilt visibility depth-map backend
FOV_OPT_OCCLUSION_BACKEND=depthmap \
FOV_OPT_VISIBILITY_DEPTHMAP_PATH=/absolute/path/to/depthmap.proto \
/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory \
<input_dir> <output_dir> 1

# auto backend selection
# current behavior: if both ESDF and depth-map are loaded, auto picks ESDF first
FOV_OPT_OCCLUSION_BACKEND=auto \
FOV_OPT_ESDF_PATH=/absolute/path/to/esdf_map.proto \
FOV_OPT_VISIBILITY_DEPTHMAP_PATH=/absolute/path/to/depthmap.proto \
/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory \
<input_dir> <output_dir> 1
```

For the warehouse FIF depth map already in this workspace, you can use:

```bash
FOV_OPT_OCCLUSION_BACKEND=depthmap \
FOV_OPT_VISIBILITY_DEPTHMAP_PATH=/home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_depth_and_landmarks/depthmap.proto \
/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory \
<input_dir> <output_dir> 1
```


### Mapping  (step1)
During the Mapping stage, we first capture a rosbag with posed images (How to is in the 'Runs' section below), the trajectory should be for  "mapping" purpose. By running both lidar localizaiton, and D455 cameras mounted on top of robot.


make a new  `<base_dir>` base directory, then place the captured rosbag into it. The rosbag contains posed images, and it should have both /Image and /Localization topics. 


The following command will extract from the bag and reconsutruct the visual features environment in COLMAP

```
./mapping.sh <base_dir> <subsampling_interval> <pytorch conda activate env name >
```

## Camera Intrinsic Error Description
running `./mapping.sh` from above, it eventually makes a call
```
python ~/colmap_utils/reconstruct_from_known_poses.py ./ --img_to_colmap_cam_list ./img_nm_to_colmap_cam.txt --img_to_colmap_pose_list ./img_name_to_colmap_Tcw.txt  --overwrite_db
```
Where in `<base_dir>`, the `./img_nm_to_colmap_cam.txt ` file supplies intrinsic parameter with a line such as

```
00000.png OPENCV 1280 720 643.9798583984375 643.3751831054688 647.8587646484375 373.6006164550781 -0.054810039699077606 0.0642876848578453 -5.264517676550895e-05 0.0007217184756882489
```
Which specified an image with `OPENCV` camera model, dimension `1280 720` and `643.9798583984375 643.3751831054688 647.8587646484375 373.6006164550781 -0.054810039699077606 0.0642876848578453 -5.264517676550895e-05 0.0007217184756882489` being `fx,fy,cx,cy,k1,k2,p1,p2` respectively 

But after running `./mapping.sh`, when constructing the map, stdout has the following

```
  Name:            00000.png
  Dimensions:      1280 x 720
  Camera:          #1 - SIMPLE_RADIAL
  Focal Length:    1536.00px
  Features:        9120

```

which means the camera intrinsics aren't registered correctly. `Dimensions: ` are correct, but the camera model `Camera:` is not. `Focal Length:` is also wrong.





## Planning (step2)
During the planning stage, we first let spot go through a user define "registrtation" trajectory. For example, a straight line across an room. we capture the posed images same as like in Mapping, let's called this beg registration_beg_1

We make a new `<reg_dir>` , place the captured registration_beg_1 into it, then run 

```
./planning.sh <base_dir> <reg_dir> <reg_sub_sampling_interval>
```

This step will extract, and plan the optimal FOV rotation trajectory based on our based visual feature environment in COLMAP and the translational trajecotry recorded in this run 

you should find `trajectory.txt` in the directory after. This file will be needed in the next step. Modify the launch file to point to this file

## Visualize Planning Result and COLMAP

After running the planning script above, visualize the planned trajectory, and the 3D pointcloud of the environment. Use `<base_dir>` as the directory of where COLMAP was built in mapping

```
python analyze_point_uncertainty.py <base_dir>
```



## Evaluation Visual Localization Error (step 3)


collect a second time, let's called it registration_beg_2, by running spot through the same trajectory again via Autowalk, but this time with gimbal controller launched, the gimbal controller will have the above trajectory file loaded.


after collection, make a new directory `<reg_2_dir>`, place the registration_beg_2 in it,

```
./evaluate_visual_localization.sh <base_dir> <reg_2_dir> <reg_sub_sampling_interval> <pytorch_env_name>
```

This script will evaluate image registration error

one can also use `<reg_dir>` in the above command, to evalaute regisration_beg_1's image registration accuracy



## Runs

### Run1:
This run is the precursor of this library. We do this so that the library is able to get localization data.

We have two launch files on spot, one for localizaiton/image rosbag recording. The other for gimbal control with a preloaded `trajectory.txt`

[This repo](https://github.com/droneslab/fio_rosws.git) is used to run FAST-LIO lidar mapping and localization. It is intended to support the [FOV Optimization main rep](https://github.com/yuyangch/FOV_Optimization_On_Manifold).

To run lidar mapping i.e. generate a PCD map file using FAST-LIO use the following:

``
roslaunch fio-main run.launch mapping_mode:=1
``

The generated map is stored in `src/fast-lio-optimization/PCD/scans.pcd`. A subsequent run of the mapping will overwrite `scans.pcd`. To save a map for later use, please make a copy of it.

### Run2
This run is the collection run for the bag. For run 2 trajectory.txt is not required. Neither is gimbal control.
To run localization i.e. use a saved PCD map and publish odometry with FAST-LIO, use the following:

`src/fast-lio-localization/PCD/scans.pcd` will be usd for localization. Ensure the right maps is copied over to this.

To capture frames from the realsense camera for COLMAP and for FOV optimization runs, use `debug.launch` like so:

``
roslaunch fio-main debug.launch
``

### Run3
This run is the collection of data post creation of trajectory.txt. This is the validation run. Here we require to place the trajectory.txt in the launch file as mentioned above. This way the gimbal will know the pitch and yaw required for this experiment.
Repeat the same steps as Run2.

`src/fast-lio-localization/PCD/scans.pcd` will be usd for localization. Ensure the right maps is copied over to this.

To capture frames from the realsense camera for COLMAP and for FOV optimization runs, use `debug.launch` like so:

``
roslaunch fio-main debug.launch
``
