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


class map_publisher:
    def __init__(self, map_folder_name,sub_sampling_interval,segmented=False):
        self.map_folder_name = map_folder_name
        pointcloud_files=None
        if not segmented:
            rospy.init_node('map_publisher', anonymous=True)
            pointcloud_files = glob.glob(f"{self.map_folder_name}/pointcloud.txt")
            self.pub_topic_name="/republished_world_frame_pointcloud_topic"
        else:
            rospy.init_node('map_publisher_segmented', anonymous=True)
            pointcloud_files = glob.glob(f"{self.map_folder_name}/*segmented_pointcloud.txt")
            self.pub_topic_name="/republished_world_frame_pointcloud_topic_segmented"
        #subsampling interval
        
        self.sub_sampling_interval=int(sub_sampling_interval)
        ## acquires bag file name
        #self.bag_file_name=None
        
        #self.directory_path = self.map_folder_name
        
        print("self.map_folder_name",self.map_folder_name)
        self.pointcloud_filename=None
        if len(pointcloud_files) == 1:
            # If there is only one .bag file, get its name
            self.pointcloud_filename = pointcloud_files[0].split("/")[-1]  # Adjust the split logic based on your OS
            print("The file name is:", self.pointcloud_filename)
        else:
            print("There is either no pointcloud.txt file or more than one pointcloud.txt file in the directory.")


        #save trajectory

        self.pointcloud_file=open(self.map_folder_name+"pointcloud.txt","r")
        #Ros Pointcloud 2 re-publisher to example lidar rotational calibration results
        #rospy.init_node('pointcloud_listener', anonymous=True)
        # Publisher to republish the received PointCloud2 message
        self.pub_world_frame = rospy.Publisher(self.pub_topic_name, PointCloud2, queue_size=1)
        #self.rate = rospy.Rate(15) 
        self.pointcloud_list=[]
        self.import_pointcloud()
        try:
            self.publish_pointcloud()
        except rospy.ROSInterruptException:
            return


    def publish_pointcloud(self):
        modified_msg = PointCloud2()
        #modified_msg.header=Header()
        modified_msg.header.frame_id = "map"
        modified_msg.header.seq = 0
        modified_msg.header.stamp = rospy.Time.now()
        print("len(self.pointcloud_list)",len(self.pointcloud_list))
        pc2_msg=pc2.create_cloud_xyz32(modified_msg.header,self.pointcloud_list)
        while(True):
            time.sleep(15)
            self.pub_world_frame.publish(pc2_msg)

    def import_pointcloud(self):
        counter=0
        with open(self.map_folder_name+self.pointcloud_filename, "r") as file:
            for line in file:
                if not counter%self.sub_sampling_interval==0:
                    counter+=1
                    continue
                x, y, z = line.strip().split(" ")  # Assuming values are separated by commas
                x = float(x)  # Convert x to float
                y = float(y)  # Convert y to float
                z = float(z)  # Convert z to float
                # Do something with x, y, and z here
                #print(f"x: {x}, y: {y}, z: {z}")  # For demonstration, printing the values
                self.pointcloud_list.append([x,y,z])
                counter+=1
if __name__=="__main__":
    importer=None
    print("len(sys.argv)",len(sys.argv))
    if len(sys.argv) == 4:
        map_folder_name = sys.argv[1]
        sub_sampling_interval=sys.argv[2]
        segmented=bool(int(sys.argv[3]))
        print("segmented is ",segmented)
        importer=map_publisher(map_folder_name+"/",sub_sampling_interval,segmented)
    else:
        print("Must provide 3 argument that is the folder name where the map is residing, subsampling interval, and whether to use segmented pointcloud")