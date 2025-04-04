#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3

import serial
import time
import sys

class python_serial_sender:
    def __init__(self):
        self.nh=None
        self.topic_name=None
        
        self.serial_port = '/dev/ttyUSB0'  # Change this to your specific serial port
        self.baud_rate = 38400
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except e:
            print(e)
        self.vector3_subscriber()


    def vector3_callback(self,data):
        # This function will be called whenever a new message is received on the topic
        rospy.loginfo("Received Vector3 message: x=%f, y=%f, z=%f", data.x, data.y, data.z)
        yaw=data.x
        if(yaw>50):
            yaw=50
        if(yaw<-50):
            yaw=50
        pitch=data.y
        data_to_send = str(data.x)+','+str(data.y)+'\n'
        # Send data
        #serial_port.write(data_to_send.encode('utf-8'))
        self.ser.write(data_to_send.encode())

    def vector3_subscriber(self):
        # Initialize the ROS node
        self.nh=rospy.init_node('vector3_subscriber', anonymous=True)

        # Define the topic to subscribe to and the message type
        self.topic_name = '/cam_motor_control'
        rospy.Subscriber(self.topic_name, Vector3, self.vector3_callback)
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        pss=python_serial_sender()
    except rospy.ROSInterruptException:
        pass









# import serial
# import time
# # Configure the serial port
# ser= serial.Serial('/dev/ttyS0', 38400,timeout=1)  # Replace 'COM1' with your actual serial port and 9600 with your baud rate


# Create a serial object







# #/dev/ttyS0, /dev/ttyS1
# if len(sys.argv) < 3:
#     print("Usage: python your_script.py yaw pitch")
#     sys.exit(1)

# arg1 = sys.argv[1]
# arg2 = sys.argv[2]

# print("yaw:", arg1)
# print("pitch:", arg2)


# data_to_send = arg1+','+arg2+'\n'
# # Send data
# #serial_port.write(data_to_send.encode('utf-8'))
# ser.write(data_to_send.encode())