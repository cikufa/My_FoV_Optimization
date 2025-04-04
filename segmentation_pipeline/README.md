# Intro
This doc describe the binaries building, manual/offline data recording, planning and analysis procedure 

# Build the manifold planning binaries
We need to build the hard coded `./Manifold_cpp/build/manifold_test_trajectory` binary that will be refer to later.

Refer to `./Manifold_cpp/CmakeLists.txt` for more details

The file to edit is `Manifold_cpp/trajectory_optimization_on_manifold/trajectory_optimizer.h`
There several tunable parameters. I will list them by line here

```
528:float step_size=(10/(this->points_list.size()*M_PI*2));
624:int max_iteration=30;
630:double ks=15;
631:float visibility_angle=15.0; #describes half of the angle of FOV conic
```
This procedure is bad and can use a config file

The main variable to tune is the step size at Line 528.

After tuning, we have the standard build procedure for c++ with CMake
```
cd ./Manifold_cpp/
mkdir build
cd ./build
cmake ..
make
```



# data recording
## run 1. 
Building a lidar map, so that in run2, odometry can be enable and the /localization topic can be recorded. Refer to Charu/Elton for the details. The main document [here](https://github.com/yuyangch/FOV_Optimization_On_Manifold?tab=readme-ov-file#running-two-launch-files-on-spot) describes the steps for this run. We use `roslaunch fio-main run.launch mapping_mode:=1` here. 

## run 2. 
Enable odometry with the map built in run1. Record image AND lidar AND `/localization` topics

We use `roslaunch fio-main run.launch mapping_mode:=0` here, to provide the `/localization` topic. 

Reference to `reference_lidar_image_and_gimbal_topics` document for the list of required image(`/camera`), lidar(`/ouster`) and localization(`/localization`) topics. Recorders must make sure the required topics are in place

There is a data processing step , to generate a trajectory `trajectory.txt`.  Refer to data processing below. 

## run 3. 
1. With the trajectory `trajectory.txt` generated, load it into spot. Elton and Charu should have more information on this traj loading step. The main documentation briefly described this trajectory loading step [here](https://github.com/yuyangch/FOV_Optimization_On_Manifold?tab=readme-ov-file#planning-step2yuyang)

Activate the gimbal!

Record all topics in `reference_lidar_image_and_gimbal_topics` just as before, but this time with the additional `/cam_motor_control` topic. Again, the recorder must make sure all these topics are recorded.

Then we can anlyze and compare the results from run3 against run2, for improvement of subject detections. 



# Data Processing

Use for `set2_curve_static_human_2024-04-15-20-15-51.bag` practice. One can find it [here] (Need UBBox access)(https://buffalo.app.box.com/folder/259833916152?s=qkxgzpewjw0cs4qqmnhd78jx53fd2c5e)

## Run 1
For questions, go to Elton and Charu for references here.
We need to save a seperate copy of `src/fast-lio-optimization/PCD/scans.pcd`, as described [here](https://github.com/yuyangch/FOV_Optimization_On_Manifold?tab=readme-ov-file#running-two-launch-files-on-spot)

## Run 2
For semantic segmentation and trajectory planning. In these procedures, we will first build the pointcloud map, then manually segment them, then plan trajectories with manual segmentation.  

0. First, download the relevant run 2  rosbag. Put it into `<base_dir>` at the project root. Then open the relevant 5 terminals below:

1.
```
roscore
```
2.
```
rosrun rviz rviz -d ./lidar_viz.rviz
```
3. Map builder, absolute path , the last argument is subsampling interval
```
python map_builder.py <base_dir> 10
```
Note: usually there is a few degrees of x/y axis offset due to how the lidar is mounted on spot. Optionally, for the best planning results. One needs to manually tune these by observing the pointcloud in rviz. this procedure is not ideal and can benefit from using a config file.  Reference line 106 of this script.

```
        #simple lidar calibration/rebase parameters set2 split side
        x_rotation_radian=(1.0/180.0)*np.pi #degrees
        y_rotation_radian=(-2.0/180.0)*np.pi #degrees
        z_rotation_radian=(0.0/180.0)*np.pi #degrees

```

4. Visualize Map,  the last argument is whether to visualize segmented map instead
```
python map_publisher.py <base_dir> 10 0
```
You should be able to see the full pointcloud after some waiting

5. Manual segmentation procedure:
The `segmentation_tools.py` is for publishinbg manual semantic segmentation "bounding circle" in the xy plane. 

First add the entry below, similar to line 165. Edit each row of `positions` with a centre of an xy sphere bounding shape. you can find this centre by holding down mouse wheel, and move the center cursor onto the target, then observe the `Focal Point:X,Y,Z` in the `views` panel (usually located to the right), in rviz.
(procedurally this is quite bad. Need to improve the workflow here by using a config file instead)
```
        elif(name=="<base_dir>"):
            positions=[#[-1.50,-1.44,0.0], #human
                       [10.64,3.557,0.0],
                       [14.293,3.49187,0.0],
                        ]
``` 

Now run the the script.

```
python segmentation_tools.py <base_dir> 
```
you should now be able to see the square boxes representing the location of the xy-plane bounding circle

A new file call `segmented_pointcloud.txt` is now generated in <base_dir>. You can visualize the segmented pointcloud another window open, and a simialr call as step 4 above 

```
python map_publisher.py <base_dir> 1 1
```
You should be able to see the segmented result as in ![reference image](reference_image.png "reference")


6. run the planning script 
```
./planning_segmentation.sh <base_dir>+/ 10
```
You should be able to see the planning result. Reference ![here](planning_result_reference.png "planning reference")

Close the 2 planning results windows that pops up after inspection

note: add `/` to `base_dir` at the end, otherwise you will find the output trajectory at repo root. 

Now the trajectory file `output_trajectory` should be generated in <base_dir>. copy and load it into spot as `trajectory`. Proceed to run 3


7. Detection:
Edit `detection_workorder.sh` to run detection on a <base_dir>. and refer to the relevent `detection.sh` and `Detection` Folder, one must download `coco.names`,`yolov3.cfg`,`yolov3.weights` into the `Detection`Folder. You can find the files [here (need UBbox access)](https://buffalo.app.box.com/folder/275203437404).

Edit `cv2_detect.py` line 36, to point to the absolute path of above 3 files. Again this is bad procedurally and can use a config file

```
    def initialize_model(self,yolo_weight_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/yolov3.weights",yolo_cfg_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/yolov3.cfg",coco_name_path="/media/chen/T7_cam_2_3/FOV_DIR/FOV_Optimization_On_Manifold_/Detection/coco.names",debug=False):
```
Now you can find the detection results in <base_dir>
`detection_result.txt` for per frame detection and `detection_result_sum.txt` for total detection. A folder named `images_with_detection` is also generated and should show the images. reference the images for manual intepretation of true/false positives


* To practice this step, use `set2_curve_static_human_2024-04-15-20-15-51.bag` as reference. Make a <base_dir> with the name `set2_curve_static_human_2024-04-15-20-15-51`

You can reference the result with this youtube link: [deactivated](https://www.youtube.com/watch?v=KDry2AzZoqQ) 



## Run 3

No need to perform trajectory planning here. Only detection is needed for comparison.

0. First, download the relevant run 3  rosbag. Put it into `<ref_dir>` at the project root. Then run the same detection step as above. Just add `<ref_dir>` into `detection_workorder.sh`


1. One can compare the trajectories with `map_publisher.py`, with the original trajectory. See a reference ![here](planning_results_comparison.png "planning results comparison") 