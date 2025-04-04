#read rosbag, build pointcloud map
import rosbag
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
#from sensor_msgs.msg import Image
import time
#from sensor_msgs.msg import CompressedImage
import json
import sys
import glob
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovariance
#from sensor_msgs.msg import CameraInfo
import os
from scipy.spatial.transform import Rotation as R

#pointclouds
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
from geometry_msgs.msg import Point32

from tf.transformations import quaternion_matrix

def euler_rotation_matrix(theta_x, theta_y, theta_z):
    """
    Generate a 3D Euler rotation matrix.
    
    Parameters:
        theta_x (float): Angle of rotation about the X-axis in radians.
        theta_y (float): Angle of rotation about the Y-axis in radians.
        theta_z (float): Angle of rotation about the Z-axis in radians.
        
    Returns:
        np.matrix: Euler rotation matrix representing the composition of rotations
                   about the X, Y, and Z axes.
    """
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, np.cos(theta_x), -np.sin(theta_x)],
                   [0.0, np.sin(theta_x), np.cos(theta_x)]])
    
    Ry = np.array([[np.cos(theta_y), 0.0, np.sin(theta_y)],
                   [0.0, 1.0, 0.0],
                   [-np.sin(theta_y), 0.0, np.cos(theta_y)]])
    
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0.0],
                   [np.sin(theta_z), np.cos(theta_z), 0.0],
                   [0.0, 0.0, 1.0]])
    
    return np.matrix(Rz @ Ry @ Rx)

class fov_bag_importer:
    def __init__(self, map_folder_name,sub_sampling_interval):
        rospy.init_node('map_builder', anonymous=True)
        #subsampling interval
        self.sub_sampling_interval=int(sub_sampling_interval)
        ## acquires bag file name
        self.bag_file_name=None
        self.map_folder_name = map_folder_name
        directory_path = self.map_folder_name
        bag_files = glob.glob(f"{directory_path}/*.bag")
        print("self.map_folder_name",self.map_folder_name)
        if len(bag_files) == 1:
            # If there is only one .bag file, get its name
            self.bag_file_name = bag_files[0].split("/")[-1]  # Adjust the split logic based on your OS
            print("The file name is:", self.bag_file_name)
        else:
            print("There is either no .bag file or more than one .bag file in the directory.")
        self.pose=Odometry()


        #save trajectory
        self.trajectory_file=open(self.map_folder_name+"trajectory.txt","w")
        self.pointcloud_file=open(self.map_folder_name+"pointcloud.txt","w")
        #Ros Pointcloud 2 re-publisher to example lidar rotational calibration results
        #rospy.init_node('pointcloud_listener', anonymous=True)
        # Publisher to republish the received PointCloud2 message
        self.pub_lidar_frame = rospy.Publisher("/republished_lidar_frame_pointcloud_topic", PointCloud2, queue_size=1)
        self.pub_world_frame = rospy.Publisher("/republished_world_frame_pointcloud_topic", PointCloud2, queue_size=1)
        #save worldframe pointcloud

        ##make new directory 
        # # Specify the path for the new directory
        # self.images_directory_path = "./"+self.map_folder_name+"/images"
        # # Specify the path for the pose/intrinsic files
        # self.img_name_to_colmap_Tcw="./"+self.map_folder_name+"img_name_to_colmap_Tcw.txt"
        # self.img_nm_to_colmap_cam="./"+self.map_folder_name+"img_nm_to_colmap_cam.txt"
        # self.name_to_tcw_file=open(self.img_name_to_colmap_Tcw,"w")
        # self.name_to_cam_intrinsic_file=open(self.img_nm_to_colmap_cam,"w")
        # Use the os.makedirs() function to create the directory and its parent directories if they don't exist
        # os.makedirs(self.images_directory_path, exist_ok=True)




        #simple lidar calibration/rebase parameters set6 v3 
        x_rotation_radian=(-3.0/180.0)*np.pi #degrees
        y_rotation_radian=(-2.5/180.0)*np.pi #degrees
        z_rotation_radian=(0.0/180.0)*np.pi #degrees


        #simple lidar calibration/rebase parameters set2 split side
        x_rotation_radian=(1.0/180.0)*np.pi #degrees
        y_rotation_radian=(-2.0/180.0)*np.pi #degrees
        z_rotation_radian=(0.0/180.0)*np.pi #degrees



        self.calibration_rotaion_matrix=euler_rotation_matrix(x_rotation_radian,y_rotation_radian,z_rotation_radian)


        #compression parameter
        self.COMPRESSED=False
        #image counter
        # self.image_counter=0
        #import bag starts here
        try:
            self.import_bag()
        except rospy.ROSInterruptException:
            pass
        
        #rospy.spin()

    def lidar_frame_transform(self,input_point):
        return self.calibration_rotaion_matrix*input_point
    def world_frame_transform(self,input_point):
        input_point=self.calibration_rotaion_matrix*input_point
        return self.odom_translation_vector+self.odom_rotation_matrix*input_point

    def build_and_publish_pointcloud2_message(self,input_msg,input_framename,ros_pubisher):
        # Create a new PointCloud2 message to store the modified points
        modified_msg = PointCloud2()
        modified_msg.header = input_msg.header
        print(modified_msg.header)
        modified_msg.height = input_msg.height
        print("modified_msg.height",modified_msg.height)
        modified_msg.width = input_msg.width
        print("modified_msg.width",modified_msg.width)
        modified_msg.fields = input_msg.fields
        print("modified_msg.fields",modified_msg.fields)
        modified_msg.is_bigendian = input_msg.is_bigendian
        print("modified_msg.is_bigendian",modified_msg.is_bigendian)
        modified_msg.point_step = input_msg.point_step
        print("modified_msg.point_step",modified_msg.point_step)
        modified_msg.row_step = input_msg.row_step
        print("modified_msg.row_step",modified_msg.row_step)
        modified_msg.is_dense = input_msg.is_dense
        print("modified_msg.is_dense",modified_msg.is_dense)

        #print("modified_msg.header.frame_id",modified_msg.header.frame_id)
        #modify frame name
        modified_msg.header.frame_id = input_framename
        # List to store the modified points
        modified_points = []
        
        # Iterate through each point in the point cloud
        for point in pc2.read_points(input_msg, field_names=("x", "y", "z"), skip_nans=True):
            # Modify the x, y, z coordinates as desired
            x, y, z = point
            #print("x,y,z",x,y,z)
            point_transformed=None
            if input_framename=="os_sensor":
                point_transformed=self.lidar_frame_transform(np.matrix([x,y,z]).transpose())
            elif input_framename=="map":
                point_transformed=self.world_frame_transform(np.matrix([x,y,z]).transpose())
                self.pointcloud_file.write(str(point_transformed[0,0])+" "+str(point_transformed[1,0])+" "+str(point_transformed[2,0])+"\n")
            #else
                #nothing, append None will cause hard fatal error 
            # Append the modified point to the list
            modified_points.append([point_transformed[0,0],point_transformed[1,0],point_transformed[2,0]])
        pc2_msg=pc2.create_cloud_xyz32(modified_msg.header,modified_points)
        # print("[build_and_publish_pointcloud2_message]")
        # print(pc2_msg.header)
        # print(pc2_msg.fields)
        # # Convert the modified points to Point32 messages and add them to the modified PointCloud2 message
        # for modified_point in modified_points:
        #     modified_point_msg = Point32()
        #     modified_point_msg.x = modified_point[0]
        #     modified_point_msg.y = modified_point[1]
        #     modified_point_msg.z = modified_point[2]
        #     modified_msg.points.append(modified_point_msg)
        
        # Publish the modified PointCloud2 message
        ros_pubisher.publish(pc2_msg)


    def import_bag(self):
        bag = rosbag.Bag(self.map_folder_name+"/"+self.bag_file_name) #<<
        # color_topic_name=""
        # if self.COMPRESSED==False:
        #     color_topic_name="/camera/color/image_raw"                  
        # else:
        #     color_topic_name="/camera/color/image_raw/compressed"
        # color_meta_data_topic_name="/camera/color/metadata"
        # color_camera_info_topic_name="/camera/color/camera_info"
        # color_camera_info_file_written=False
        odometry_topic_name="/localization"
        pointcloud_topic_name="/ouster/points"
        counter=0
        for topic, msg, t in bag.read_messages():
            # print("t is", t)
            if(topic==odometry_topic_name):
                self.pose=msg.pose
                self.trajectory_file.write(str(self.pose.pose.orientation.w)+" "+str(self.pose.pose.orientation.x)+" "+str(self.pose.pose.orientation.y)+" "+str(self.pose.pose.orientation.z)+" "+str(self.pose.pose.position.x)+" "+str(self.pose.pose.position.y)+" "+str(self.pose.pose.position.z)+"\n")
                # print("pose updated to",self.pose)
                # Extract quaternion and position from the Odometry message
                orientation = msg.pose.pose.orientation
                position = msg.pose.pose.position
                
                # Convert quaternion to rotation matrix
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                self.odom_rotation_matrix = np.matrix(quaternion_matrix(quaternion)[:3, :3])
                
                # Extract position
                self.odom_translation_vector = np.matrix([position.x, position.y, position.z]).transpose()
                
                # # Print the rotation matrix and translation vector
                # #rospy.loginfo("Rotation Matrix:")
                # rospy.loginfo(self.odom_rotation_matrix)
                # rospy.loginfo(type(self.odom_rotation_matrix))
                # #rospy.loginfo("Translation Vector:")
                # rospy.loginfo(self.odom_translation_vector)
                # rospy.loginfo(type(self.odom_translation_vector))
            if(topic==pointcloud_topic_name ):
                if not counter%self.sub_sampling_interval==0:
                    counter+=1
                    continue
                rospy.loginfo("Received a PointCloud2 message")
                #re-published sensorframe
                self.build_and_publish_pointcloud2_message(msg,"os_sensor",self.pub_lidar_frame)
                #re-publish world frame
                self.build_and_publish_pointcloud2_message(msg,"map",self.pub_world_frame)
                counter+=1

                #save world frame
        bag.close()
        self.trajectory_file.close()
        self.pointcloud_file.close()

if __name__=="__main__":
    importer=None
    print("len(sys.argv)",len(sys.argv))
    if len(sys.argv) == 3:
        map_folder_name = sys.argv[1]
        sub_sampling_interval=sys.argv[2]
        print()
        importer=fov_bag_importer(map_folder_name+"/",sub_sampling_interval)
    else:
        print("Must provide 2 argument that is the folder name where the bag is residing, and sub sampling interval. Create a map folder and put the bag in it if you haven't")