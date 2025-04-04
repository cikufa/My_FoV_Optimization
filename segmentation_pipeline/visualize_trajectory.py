#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
import sys  
from math import atan2, sin, cos
def euler_to_quaternion(yaw):
    """
    Convert yaw angle to Quaternion
    """
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(yaw / 2)
    quaternion.w = cos(yaw / 2)
    return quaternion

def xy_to_yaw(x, y):
    """
    Convert x, y coordinates to yaw angle (in radians)
    """
    return atan2(y, x)


def publish_odometry_from_file(file_path, topic_name):
    rospy.init_node('odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher(topic_name, Odometry, queue_size=10)
    rate = rospy.Rate(20)  # Adjust the publishing rate as needed
    
    with open(file_path, 'r') as file:
        counter=0
        for line in file:
            counter+=1
            if counter%3==0:
                try:
                    parts = line.strip().split(',')
                    x, y, z = map(float, parts[:3])
                    v_x, v_y, v_z = map(float, parts[3:])
                    

                    
                    odom_msg = Odometry()
                    odom_msg.header.stamp = rospy.Time.now()
                    odom_msg.header.frame_id = "map"
                    odom_msg.child_frame_id = "base_link"
                    odom_msg.pose.pose.position = Point(x, y, z)
                    odom_msg.pose.pose.orientation = euler_to_quaternion(xy_to_yaw(v_x,v_y))
                    odom_msg.twist.twist.linear = Vector3(v_x, v_y, v_z)
                    
                    odom_pub.publish(odom_msg)
                    rate.sleep()
                except Exception as e:
                    rospy.logerr("Error parsing line: %s, Error: %s", line, str(e))

if __name__ == '__main__':
    importer=None
    print("len(sys.argv)",len(sys.argv))
    if len(sys.argv) == 2:
        map_folder_name = sys.argv[1]
    else:
        print("Must provide 1 argument that is the folder name where the map is residing")
    try:
        file_path = map_folder_name+"output_trajectory.txt"  # Change this to your file path
        topic_name = "/planned_trajectory"  # Change this to your desired topic name
        publish_odometry_from_file(file_path, topic_name)
    except rospy.ROSInterruptException:
        pass
