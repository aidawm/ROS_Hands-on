#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import numpy as np

class Bug():


    def __init__(self):
        
        rospy.init_node('controller_node', anonymous=True)
        
        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        self.goal_point_x, self.goal_point_y = 3, -1
        self.epsilon_angular = 0.1
        self.epsilon_linear = 0.1
        self.distances_from_wall = 0.4

        self.GO , self.ROTATE, self.FOLLOW_WALL_ROTATE, self.FOLLOW_WALL_GO = 0, 1, 2,3

        self.state = self.ROTATE

        self.v_linear = 0.2
        
        self.Kp = 0.6
        self.Kd = 10


    def odometry_callback(self, msg):
        # Get robot's current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))


    def distances_around_robot(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        self.laser_ranges = laser_data.ranges[:]
        right_rng = laser_data.ranges[45:135]
        forward_rng = laser_data.ranges[355:360] + laser_data.ranges[0:5]

        rospy.loginfo(f"right = {laser_data.ranges[270]}, left = {laser_data.ranges[90]}")

        min_right = right_rng[0]
        min_index = 0
        for i, rng in enumerate(right_rng):
            if rng< min_right:
                min_right = rng
                min_index = i

        return (min(self.laser_ranges[0:115]),min_index+45), min(forward_rng)
    
    def euclidean_distance(self, x1, x2, y1, y2):
        point1 = np.array((x1, y1))
        point2 = np.array((x2, y2))
        return np.linalg.norm(point1 - point2)
    
    
    def calculate_rotation_angle(self):
        self.goal_orientation = math.atan2((self.goal_point_y - self.current_y), (self.goal_point_x - self.current_x))

        if self.current_yaw > 0:
            sign = -1 if (self.current_yaw - math.pi < self.goal_orientation < self.current_yaw) else +1
        else:
            sign = +1 if (self.current_yaw + math.pi > self.goal_orientation > self.current_yaw) else -1
        angular_err = sign * (math.pi - abs(abs(self.current_yaw - self.goal_orientation) - math.pi))

        # rospy.loginfo(f"error  = {}")
        return angular_err


    
    def run(self):
        while not rospy.is_shutdown():
            d_right , d_forward = self.distances_around_robot()

            rospy.loginfo(f"state = {self.state}")
            
            if self.state != self.FOLLOW_WALL_ROTATE and self.state != self.FOLLOW_WALL_GO:
                # rospy.loginfo(f"d_forward = {d_forward}")
                if d_forward <= self.distances_from_wall:

                    self.state = self.FOLLOW_WALL_ROTATE

                    self.prev_error = 0

                    self.cmd_vel_pub.publish(Twist())
                    rospy.sleep(2)

                    rotate = self.current_yaw - math.pi/2
                    for i in range(4):
                        orientation = i * math.pi/2
                        if abs(rotate + orientation) < math.pi/4:
                            self.next_robot_yaw = -orientation
        

                    rospy.loginfo(f"next_yaw = {self.next_robot_yaw}")



    
            if self.state == self.GO:

                distance_from_goal = self.euclidean_distance(self.current_x, self.goal_point_x, self.current_y, self.goal_point_y)
                if distance_from_goal <= self.epsilon_linear:
                    self.cmd_vel_pub.publish(Twist())
                    rospy.sleep(2)
                    break

                twist = Twist()
                rospy.loginfo(min(self.v_linear, d_forward-self.distances_from_wall))
                twist.linear.x = min(self.v_linear, (d_forward-self.distances_from_wall)/2)
                self.cmd_vel_pub.publish(twist)
                # rospy.sleep(2)
                continue

            if self.state == self.ROTATE:
                self.error_angle = self.calculate_rotation_angle()
                
                if abs(self.error_angle) <= self.epsilon_angular:
                    self.cmd_vel_pub.publish(Twist())
                    rospy.sleep(2)

                    self.state = self.GO
                    continue

                twist = Twist()
                twist.angular.z = self.Kp*self.error_angle
                self.cmd_vel_pub.publish(twist)

                
                

            if self.state == self.FOLLOW_WALL_ROTATE:
    
                if self.current_yaw > 0:
                    sign = -1 if (self.current_yaw - math.pi < self.next_robot_yaw < self.current_yaw) else +1
                else:
                    sign = +1 if (self.current_yaw + math.pi > self.next_robot_yaw > self.current_yaw) else -1
                angular_err = sign * (math.pi - abs(abs(self.current_yaw - self.next_robot_yaw) - math.pi))

                if abs(angular_err) <= self.epsilon_angular:
                    self.cmd_vel_pub.publish(Twist())
                    rospy.sleep(2)

                    self.state = self.FOLLOW_WALL_GO
                    continue
                rospy.loginfo(f"ANGULAR ERROR= {angular_err}")
                twist = Twist()
                twist.angular.z = self.Kp*angular_err
                self.cmd_vel_pub.publish(twist)

            
            if self.state == self.FOLLOW_WALL_GO:

                rotation_angle = self.calculate_rotation_angle()
                prev_error = 0
                # while -1*self.laser_ranges[int(math.degrees(rotation_angle))] <= 2* self.distances_from_wall:
                while not (min(self.laser_ranges[int(math.degrees(rotation_angle))-10:int(math.degrees(rotation_angle))+10]) > 1.2 and abs(math.degrees(rotation_angle))>=0 and abs(math.degrees(rotation_angle))<=90):
                    # rospy.loginfo(f"rotation_angle = {int(math.degrees(rotation_angle))}")
                    # rospy.loginfo(f"laser distance= {self.laser_ranges}")
                    # rospy.loginfo(f"angle distance = {self.laser_ranges[int(math.degrees(rotation_angle))]}")
                    d_right , d_forward = self.distances_around_robot()
                    err = d_right[0] - self.distances_from_wall

                    P = self.Kp * err
                    D = 0#self.Kd * (err - prev_error)
                    # rospy.loginfo(f"P = {P}, D={D}")
                    twist = Twist()
                    twist.angular.z = P + D

    
                    if abs(twist.angular.z) > math.radians(5):
                        twist.linear.x = self.v_linear/2
                        # twist.angular.z
                    else:
                        twist.linear.x = self.v_linear
                    prev_error = err         

                    # rospy.loginfo(f"error : {err} speed : {twist.linear.x} theta : {twist.angular.z}")

                    # d = self.distance_from_wall()
                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()

                    rotation_angle = self.calculate_rotation_angle()
                
                rospy.loginfo(f"rotation_angle {-1*int(math.degrees(rotation_angle))},{ self.laser_ranges[-1*int(math.degrees(rotation_angle))]} " )
                self.cmd_vel_pub.publish(Twist())
                rospy.sleep(10)

                self.state = self.ROTATE
     


if __name__ == '__main__':
    try:
        controller = Bug()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")