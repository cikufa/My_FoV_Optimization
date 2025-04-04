#Note:March 2nd, each camera frame timestamp is the same frame time stamp reported in metadata. There are the same amount of metadata as frame. In summary use frame time stamp
#time stamps are in ms
import rosbag
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time
from sensor_msgs.msg import CompressedImage
import json
import sys
import glob
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import CameraInfo
import os



class fov_bag_importer:
    def __init__(self, map_folder_name,sub_sampling_interval):



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
        self.trajectory_file=open("./"+self.map_folder_name+"trajectory.txt","w")
        # camera model
        self.camera_model="PINHOLE" #PINHOLE or OPENCV
        # camera intrinsic
        self.camera_info_D=None
        self.camera_info_K=None
        self.camera_width=None
        self.camera_height=None
        ##make new directory 
        # Specify the path for the new directory
        self.images_directory_path = "./"+self.map_folder_name+"/images"
        # Specify the path for the pose/intrinsic files
        self.img_name_to_colmap_Tcw="./"+self.map_folder_name+"img_name_to_colmap_Tcw.txt"
        self.img_nm_to_colmap_cam="./"+self.map_folder_name+"img_nm_to_colmap_cam.txt"
        self.name_to_tcw_file=open(self.img_name_to_colmap_Tcw,"w")
        self.name_to_cam_intrinsic_file=open(self.img_nm_to_colmap_cam,"w")
        # Use the os.makedirs() function to create the directory and its parent directories if they don't exist
        os.makedirs(self.images_directory_path, exist_ok=True)


        #compression parameter
        self.COMPRESSED=False
        #image counter
        self.image_counter=0
        #import bag starts here
        self.import_bag()


    def import_bag(self):
        bag = rosbag.Bag("./"+self.map_folder_name+"/"+self.bag_file_name) #<<
        color_topic_name=""
        if self.COMPRESSED==False:
            color_topic_name="/camera/color/image_raw"                  
        else:
            color_topic_name="/camera/color/image_raw/compressed"
        color_meta_data_topic_name="/camera/color/metadata"
        color_camera_info_topic_name="/camera/color/camera_info"
        color_camera_info_file_written=False
        odometry_topic_name="/localization"

        for topic, msg, t in bag.read_messages():
            print("t is", t)
            if(topic ==color_camera_info_topic_name and color_camera_info_file_written==False):

                color_camera_info_file_written=True
                self.camera_info_D=msg.D
                self.camera_info_K=msg.K
                self.camera_width=msg.width
                self.camera_height=msg.height
                # print("camera_info acquired. K is ",self.camera_info_K,type(self.camera_info_K),type(self.camera_info_K[0]))
                # print("camera_info acquired. D is ",self.camera_info_D,type(self.camera_info_D),type(self.camera_info_D[0]))
                # print("msg is ",msg)
            if(topic == color_topic_name):
                if self.pose is None:
                    continue
                if not self.image_counter%self.sub_sampling_interval == 0:
                    self.image_counter+=1
                    continue
                formatted_number = '{:05d}'.format(self.image_counter)+".png"
                name=self.images_directory_path+"/"+formatted_number
                print ("self.image_counter",self.image_counter)
                y = np.frombuffer(msg.data, dtype=np.uint8)
                print("rgb",y.shape)
                #write image
                if self.COMPRESSED:
                    image = cv2.imdecode(y, cv2.IMREAD_COLOR)
                    cv2.imwrite(name,image)
                else:
                    cv2.imwrite(name,cv2.cvtColor(y.reshape(msg.height,msg.width,3), cv2.COLOR_RGB2BGR ))
                #write pose file
                
                #print("self.pose",self.pose)
                #print("self.pose.pose.orientation",self.pose.pose.orientation)
                self.name_to_tcw_file.write(formatted_number+" "+str(self.pose.pose.orientation.w)+" "+str(self.pose.pose.orientation.x)+" "+str(self.pose.pose.orientation.y)+" "+str(self.pose.pose.orientation.z)+" "+str(self.pose.pose.position.x)+" "+str(self.pose.pose.position.y)+" "+str(self.pose.pose.position.z)+"\n")
                #write cam intrinsic file
                fx=str(self.camera_info_K[0])
                fy=str(self.camera_info_K[4])
                cx=str(self.camera_info_K[2])
                cy=str(self.camera_info_K[5])

                k1=str(self.camera_info_D[0])
                k2=str(self.camera_info_D[1])
                p1=str(self.camera_info_D[2])
                p2=str(self.camera_info_D[3])
                width=str(self.camera_width)
                height=str(self.camera_height)
                if self.camera_model=="OPENCV":
                    self.name_to_cam_intrinsic_file.write(formatted_number+" "+"OPENCV"+" "+width+" "+height+" "+fx+" "+fy+" "+cx+" "+cy+" "+k1+" "+k2+" "+p1+" "+p2+"\n")
                elif self.camera_model=="PINHOLE":
                    self.name_to_cam_intrinsic_file.write(formatted_number+" "+"PINHOLE"+" "+width+" "+height+" "+fx+" "+fy+" "+cx+" "+cy+"\n")
                self.image_counter+=1 
            if(topic==odometry_topic_name):
                self.pose=msg.pose
                self.trajectory_file.write(str(self.pose.pose.orientation.w)+" "+str(self.pose.pose.orientation.x)+" "+str(self.pose.pose.orientation.y)+" "+str(self.pose.pose.orientation.z)+" "+str(self.pose.pose.position.x)+" "+str(self.pose.pose.position.y)+" "+str(self.pose.pose.position.z)+"\n")
                print("pose updated to",self.pose)
        bag.close()
        self.trajectory_file.close()

if __name__=="__main__":
    importer=None
    print("len(sys.argv)",len(sys.argv))
    if len(sys.argv) == 3:
        map_folder_name = sys.argv[1]
        sub_sampling_interval=sys.argv[2]
        print()
        importer=fov_bag_importer(map_folder_name,sub_sampling_interval)
    else:
        print("Must provide 2 argument that is the folder name where the bag is residing, and sub sampling interval. Create a map folder and put the bag in it if you haven't")