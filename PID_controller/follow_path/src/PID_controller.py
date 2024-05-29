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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize ROS subscriber for robot odometry
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
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


        self.epsilon = 0.2
        self.integral_x = 0.0
        self.integral_yaw = 0.0
        self.previous_error_x = 0.0
        self.previous_error_yaw = 0.0
        self.dt = 0.05
        self.back_point = False

        self.path_type = rospy.get_param("/controller/path_type")

        self.get_path_points()


    def get_path_points(self):

        if self.path_type == 0 : 
            self.X = [4]
            self.Y = [4]



        elif self.path_type  == 1:
            X1 = np.linspace(-1, 1 , 100)
            Y1 = np.array([3]*100)

            X2 = np.linspace(1, 1 + 2**(1/2) , 100)
            Y2 = - (2**(1/2)) * (X2 - 1) + 3

            Y3 = np.linspace(1, -1 , 100)
            X3 = np.array([1 + 2**(1/2)]*100)

            X4 = np.linspace(1 + 2**(1/2), 1, 100)
            Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

            X5 = np.linspace(1, -1 , 100)
            Y5 = np.array([-3]*100)

            X6 = np.linspace(-1, -1 - 2**(1/2) , 100)
            Y6 = - (2**(1/2)) * (X6 + 1) - 3 


            Y7 = np.linspace(-1, 1 , 100)
            X7 = np.array([- 1 - 2**(1/2)]*100)


            X8 = np.linspace(-1 - 2**(1/2), -1, 100)
            Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1


            self.X , self.Y = np.concatenate([X1,X2,X3,X4,X5,X6,X7,X8]), np.concatenate([Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8])


        
        elif self.path_type  == 2: 
            X1 = np.linspace(-6., -2 , 50)
            Y1 = np.zeros((50,))

            x_dim, y_dim = 2,2
            t = np.linspace(np.pi, 0, 100)
            X2 = x_dim * np.cos(t) 
            Y2 = y_dim * np.sin(t)

            X3 = np.linspace(2, 6 , 50)
            Y3 = np.zeros((50,))

            x_dim, y_dim = 6,6
            t = np.linspace(np.pi*2, np.pi, 200)
            X4 = x_dim * np.cos(t) 
            Y4 = y_dim * np.sin(t)


            self.X , self.Y = np.concatenate([X1,X2, X3 , X4]), np.concatenate([Y1,Y2,Y3,Y4])

        

        elif self.path_type  == 3:
            # Logarithmic spiral 

            a = 0.17
            k = math.tan(a)
            X , Y = [] , []

            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
                X.append(dx)
                Y.append(dy) 
            self.X = X
            self.Y = Y

        self.check_points = [0] * len(self.X)
        self.previous_target_index = -1



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


    def next_destination(self):
        my_min = 99999
        my_x = 0
        my_y = 0
        min_index = -1

        for i in range(len(self.X)):
            diff_ang = float("{:.2f}".format(np.arctan2((self.Y[i] - self.current_y), (self.X[i] - self.current_x))))
            if self.current_yaw > 0:
                sign = -1 if (self.current_yaw - math.pi < diff_ang < self.current_yaw) else +1
            else:
                sign = +1 if (self.current_yaw + math.pi > diff_ang > self.current_yaw) else -1
            diff_ang = sign * (math.pi - abs(abs(self.current_yaw - diff_ang) - math.pi))
            if -math.pi / 2 < diff_ang < math.pi / 2:
                my_distance = self.euclidean_distance(self.current_x, self.X[i], self.current_y, self.Y[i])
                if my_min > my_distance and my_distance > abs(self.epsilon):
                    my_min = my_distance
                    min_index = i
                    my_x = self.X[i]
                    my_y = self.Y[i]
        
        # no point is on the front, so find nearest point on the back
        if min_index == -1:
            for i in range(len(self.X)):
                diff_ang = float("{:.2f}".format(np.arctan2((self.Y[i] - self.current_y), (self.X[i] - self.current_x))))
                if self.current_yaw > 0:
                    sign = -1 if (self.current_yaw - math.pi < diff_ang < self.current_yaw) else +1
                else:
                    sign = +1 if (self.current_yaw + math.pi > diff_ang > self.current_yaw) else -1
                diff_ang = sign * (math.pi - abs(abs(self.current_yaw - diff_ang) - math.pi))
                my_distance = self.euclidean_distance(self.current_x, self.X[i], self.current_y, self.Y[i])
                if my_min > my_distance and my_distance > abs(self.epsilon):
                    # don't go on the previous line
                    if diff_ang > 0.1:
                        my_min = my_distance
                        min_index = i
                        my_x = self.X[i]
                        my_y = self.Y[i]

                    
        self.target_x = my_x
        self.target_y = my_y


        rospy.loginfo(f"{self.target_x}, {self.target_y}")



    def run(self):
        rospy.sleep(1)
        rate = rospy.Rate(1/self.dt)  

        if (self.path_type == 0):
            self.target_x = self.X[0]
            self.target_y = self.Y[0]


        while not rospy.is_shutdown():
            if (self.path_type !=0):

                self.next_destination()
            self.goal_orientation = float("{:.2f}".format(np.arctan2((self.target_y - self.current_y), (self.target_x - self.current_x))))

            linear_err = self.euclidean_distance(self.current_x, self.target_x, self.current_y, self.target_y)

            if (self.path_type == 0):

                if linear_err < self.epsilon:
                    self.cmd_vel_pub.publish(Twist())
                    rate.sleep()
                    break

            

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
