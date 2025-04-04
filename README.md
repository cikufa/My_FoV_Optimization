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
