#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys


def import_pointcloud(map_folder_name,positions):
    pointcloud_list=[]
    counter=0
    output_pointcloud_file=open(map_folder_name+"segmented_pointcloud.txt", "w")
    with open(map_folder_name+"pointcloud.txt", "r") as file:
        for i,line in enumerate(file):
            print("Writing line " +str(i))
            if not counter%10==0:
                counter+=1
                continue
            x, y, z = line.strip().split(" ")  # Assuming values are separated by commas
            x = float(x)  # Convert x to float
            y = float(y)  # Convert y to float
            z = float(z)  # Convert z to float
            # Do something with x, y, and z here
            #print(f"x: {x}, y: {y}, z: {z}")  # For demonstration, printing the values
            for position in positions:
                 if ((x-position[0])**2+(y-position[1])**2)<0.5**2:#radial distance on x,y
                    output_pointcloud_file.write(str(x)+" "+str(y)+" "+str(z)+"\n")
                    break;
            counter+=1;    

def publish_box(map_folder_name,positions):
    rospy.init_node('box_publisher_node', anonymous=True)
    box_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(0.25)  # 1 Hz
    #

    while not rospy.is_shutdown():
        counter=0
        for position in positions:
            box = Marker()
            box.header.frame_id = "map"  # Change this to the frame you want
            box.header.stamp = rospy.Time.now()
            box.ns = "basic_shapes"
            box.id = counter
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = position[0]
            box.pose.position.y = position[1]
            box.pose.position.z = position[2]
            box.pose.orientation.x = 0.0
            box.pose.orientation.y = 0.0
            box.pose.orientation.z = 0.0
            box.pose.orientation.w = 1.0
            box.scale.x = 1.0  # Change dimensions as needed
            box.scale.y = 1.0
            box.scale.z = 1.0
            box.color.r = 0.0
            box.color.g = 1.0
            box.color.b = 0.0
            box.color.a = 1.0
            box.lifetime = rospy.Duration()

            box_publisher.publish(box)
            counter+=1
        rate.sleep()

if __name__ == '__main__':
    print("len(sys.argv)",len(sys.argv))
    if len(sys.argv) == 2:
        positions=None
        map_folder_name=sys.argv[1]
        # name= map_folder_name
        name=map_folder_name.split("/")[-2]
        print("name is ",name)
        positions=[[11.7989,3.82,0.0], #chair
                       # [6.92,3.08,0.0],
                       # [13.306,-2.72,0.0],
                       # [16.43,0.1054,0.0],
                       # [15.22,6.36,0.0],
                       [3.95,-2.09,0.0],
                       # [16.59,6.46,0.0],
                        ]
        positions=[#[-1.50,-1.44,0.0], #human
                       [6.828,2.793,0.0],
                       #[13.82,-2.83,0.0],
                       [15.25,6.16,0.0],
                       #[15.48,-6.5,0.0],
                       [16.5437,6.13,0.0],
                        ]

        try:
            import_pointcloud(map_folder_name,positions)
            publish_box(map_folder_name,positions)
        except rospy.ROSInterruptException:
            pass
    else:
        print("must supply a map name as arg 1")
