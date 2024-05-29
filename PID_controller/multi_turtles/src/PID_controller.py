#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import tf
import rospy
from geometry_msgs.msg import Twist
import math
import numpy as np

class ControllerNode:

    def __init__(self):
        rospy.init_node('controller_node')
        self.cmd_vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        
        # Initialize ROS subscriber for robot odometry
        rospy.Subscriber('/tb3_0/odom', Odometry, self.target_odometry_callback)
        rospy.Subscriber('/tb3_1/odom', Odometry, self.odometry_callback)
        rospy.on_shutdown(self.shutdown)

        self.target_x = 0
        self.target_y = 0
        self.target_yaw = 0

        # gains for angular PID controller
        self.Kp_ang = rospy.get_param("/controller/kp_ang") #30 
        self.Ki_ang = rospy.get_param("/controller/ki_ang") #0
        self.Kd_ang = rospy.get_param("/controller/kd_ang") #15

        # gains for linear PID controller
        self.Kp_lin = rospy.get_param("/controller/kp_lin") #0.6
        self.Ki_lin = rospy.get_param("/controller/ki_lin") #0.01
        self.Kd_lin = rospy.get_param("/controller/kd_lin") #6


        self.epsilon = 1
        self.integral_x = 0.0
        self.integral_yaw = 0.0
        self.previous_error_x = 0.0
        self.previous_error_yaw = 0.0
        self.dt = 0.05
        self.previous_index = 0
        self.back_point = False


    def calculate_control_command(self, linear_err, angular_err):
        # Calculate the errors
        error_x = linear_err
        error_yaw = angular_err

        # Proportional terms
        proportional_x = self.Kp_lin * error_x
        proportional_yaw = self.Kp_ang * error_yaw

        # Integral terms
        self.integral_x += error_x * self.dt
        self.integral_yaw += error_yaw * self.dt
        integral_x = self.Ki_lin * self.integral_x
        integral_yaw = self.Ki_ang * self.integral_yaw

        # Derivative terms
        derivative_x = self.Kd_lin * (error_x - self.previous_error_x) / self.dt
        derivative_yaw = self.Kd_ang * (error_yaw - self.previous_error_yaw) / self.dt

        # Calculate the control commands
        control_command_x = proportional_x + integral_x + derivative_x
        control_command_yaw = proportional_yaw + integral_yaw + derivative_yaw

        # Update the previous errors
        self.previous_error_x = error_x
        self.previous_error_yaw = error_yaw

        return control_command_x, control_command_yaw

    def target_odometry_callback(self, msg):
        # Get robot's current position
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.target_yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    def odometry_callback(self, msg):
        # Get robot's current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    def shutdown(self):
        rospy.loginfo('Stopping the robot')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def euclidean_distance(self, x1, x2, y1, y2):
        point1 = np.array((x1, y1))
        point2 = np.array((x2, y2))
        return np.linalg.norm(point1 - point2)

    def run(self):
        rospy.sleep(1)
        rate = rospy.Rate(1/self.dt)  

        while not rospy.is_shutdown():

            self.goal_orientation = float("{:.2f}".format(np.arctan2((self.target_y - self.current_y), (self.target_x - self.current_x))))

            linear_err = self.euclidean_distance(self.current_x, self.target_x, self.current_y, self.target_y)
            if linear_err < self.epsilon:
                self.cmd_vel_pub.publish(Twist())

            if self.current_yaw > 0:
                sign = -1 if (self.current_yaw - math.pi < self.goal_orientation < self.current_yaw) else +1
            else:
                sign = +1 if (self.current_yaw + math.pi > self.goal_orientation > self.current_yaw) else -1
            angular_err = sign * (math.pi - abs(abs(self.current_yaw - self.goal_orientation) - math.pi))

            x, yaw = self.calculate_control_command(linear_err, angular_err)
            twist = Twist()
            twist.linear.x = x
            twist.angular.z = yaw
            self.cmd_vel_pub.publish(twist)
            rate.sleep() 
            



    
if __name__ == "__main__":
    
    controller = ControllerNode()
    controller.run()
