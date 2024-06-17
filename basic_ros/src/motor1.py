#!/usr/bin/python3

import rospy
from hw0.msg import *

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('motor1', anonymous=True)
    rospy.Subscriber("motor1_values", motor_value, callback)
    rospy.spin()


listener()