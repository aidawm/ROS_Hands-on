#!/usr/bin/env python

import rospy
from hw0.srv import *
from hw0.msg import * 
from collections import deque

# from __future__ import print_function


class Controller:

    def __init__(self) -> None:
        self.sensor_Service_name = "distances"
        rospy.wait_for_service(self.sensor_Service_name)
        self.pub_m1 = rospy.Publisher('motor1_values', motor_value, queue_size=1000)
        self.pub_m2 = rospy.Publisher('motor2_values', motor_value, queue_size=1000)

        self.directions = ["front","right","back","left"]
    
   

    def get_sensors_data(self):
        try:
            s = rospy.ServiceProxy(self.sensor_Service_name,proximities)
            resp = s()
            self.sensor_values = [resp.front,resp.right,resp.back,resp.left]

        except rospy.ServiceException as e: 
            print(e)

    

    def publish_motor_values(self):
        m1 = motor_value()
        m2 = motor_value()

        m1.degree =  self.m1_degree
        m1.isClockwise = self.m1_isClockwise
        m2.degree =  self.m2_degree
        m2.isClockwise = self.m2_isClockwise

        self.pub_m1.publish(m1)
        self.pub_m2.publish(m2)

    def find_nearest(self):
        min_direction = 0
        min_value = self.sensor_values[0]
    
        for i in range(4):
            v = self.sensor_values[i]

            if ( v < min_value):
                min_direction = i
                min_value = v

        return min_direction

    
    def rotate_nearest(self,direction_index): 

        min_direction = self.directions[direction_index]

        if min_direction == 'front' :
            self.m1_degree = 180
            self.m1_isClockwise = True

            self.m2_degree = 180
            self.m2_isClockwise = False
        elif  min_direction == 'right' :

            self.m1_degree = 90
            self.m1_isClockwise = False

            self.m2_degree = 90
            self.m2_isClockwise = True

        elif  min_direction == 'left' :

            self.m1_degree = 90
            self.m1_isClockwise = True

            self.m2_degree = 90
            self.m2_isClockwise = False

        else:

            self.m1_degree = 0
            self.m1_isClockwise = True

            self.m2_degree = 0
            self.m2_isClockwise = False


    def shift_sensor_data (self,degree):
        # print (degree)

        my_deque = deque(self.sensor_values)
    
        if degree == 90:
            my_deque.rotate(-1)
        
        elif degree == -90: 
            my_deque.rotate(1)

        elif degree == 180:
            my_deque.rotate(2)

        self.sensor_values = list(my_deque)
    
    def find_farthest(self):
        max_direction = 0
        max_value = self.sensor_values[0]
    
        for i in range(4):
            v = self.sensor_values[i]

            if ( v > max_value):
                max_direction = i
                max_value = v
        return max_direction
    
    def rotate_farthest(self,direction_index): 

        min_direction = self.directions[direction_index]

        if min_direction == 'front' :
            self.m1_degree = 0
            self.m1_isClockwise = True

            self.m2_degree = 0
            self.m2_isClockwise = False

        elif  min_direction == 'right' :

            self.m1_degree = 90
            self.m1_isClockwise = True

            self.m2_degree = 90
            self.m2_isClockwise = False

        elif  min_direction == 'left' :

            self.m1_degree = 90
            self.m1_isClockwise = False

            self.m2_degree = 90
            self.m2_isClockwise = True

        else:

            self.m1_degree = 180
            self.m1_isClockwise = True

            self.m2_degree = 180
            self.m2_isClockwise = False

    
    def go_forward (self, distance):
        self.m1_degree = distance
        self.m1_isClockwise = True      
        self.m2_degree = distance
        self.m2_isClockwise = True

    def controll_robot(self):
        rospy.init_node('controller', anonymous=True)
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            
            self.get_sensors_data()

            direction_index = self.find_nearest()

            self.rotate_nearest(direction_index)
            self.publish_motor_values()

            print(f" sensor data ---> {self.sensor_values}")
            print(f" nearest object is on the {self.directions[direction_index]} of the robot")
            print(f" robot should rotate {self.m1_degree} degree and clockwise is : {self.m1_isClockwise}")

            print("\n")
            # print (self.sensor_values)
            degree = self.m1_degree
            if not self.m1_isClockwise : 
                degree *= -1
            self.shift_sensor_data(degree)
            # print (self.sensor_values)
            # print("\n")

            direction_index = self.find_farthest()
            self.rotate_farthest(direction_index)
            self.publish_motor_values()
            
            print(f" current sensor data after rotation ---> {self.sensor_values}")
            print(f" farthest object is on the {self.directions[direction_index]} of the robot")
            print(f" robot should rotate {self.m1_degree} degree and clockwise is : {self.m1_isClockwise}")

            self.go_forward(self.sensor_values[direction_index]-10)
            self.publish_motor_values()

            print("\n")
            
            print(f" now robot should go forward for {self.m1_degree} and clockwise is : {self.m1_isClockwise}")
            print("\n") 
            print("-------------------\n\n")
            # self.pub_m1.publish()

            rate.sleep()


if __name__ == "__main__":
    c= Controller()
    c.controll_robot()