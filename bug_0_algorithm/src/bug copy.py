#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PIDController():


    def __init__(self):
        
        rospy.init_node('wall_follower_node', anonymous=True)
        
        self.Ki = 0.0
        self.Kp = 0.6
        self.Kd = 10
        
        self.dt = 0.1
        self.v = 0.26
        self.D = 1
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []


    
    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[225:315]
        # d = sum(rng)/len(rng)
        d = min (rng)
        return d
    
    def follow_wall(self):
        
        # d = self.distance_from_wall()    
        integral = 0
        prev_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            d = self.distance_from_wall() 

            # self.cmd_vel.publish(move_cmd)

            err = self.D - d
            integral += err * self.dt
            
            P = self.Kp * err
            I = self.Ki * integral
            D = self.Kd * (err - prev_error)
            rospy.loginfo(f"P = {P}, I = {I}, D={D}")
            move_cmd.angular.z = P + I + D

            if abs(move_cmd.angular.z) > math.radians(30):
                self.cmd_vel.publish(Twist())
                self.r.sleep()

                move_cmd.linear.x = self.v/10

            elif abs(move_cmd.angular.z) > math.radians(20):
                move_cmd.linear.x = self.v / 6
            else:
                move_cmd.linear.x = self.v
            prev_error = err         
            
            rospy.loginfo(f"error : {err} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")
            
            # d = self.distance_from_wall()
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            

if __name__ == '__main__':
    try:
        pidc = PIDController()
        pidc.follow_wall()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")