#!/usr/bin/python3

import math

# ROS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64



class Controller:

    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()
        self.lane_vel = Twist()
        self.exe_mode = 0
        self.sub_max_vel = rospy.Subscriber('/control/lane_vel', Twist, self.get_lane_info, queue_size = 1)
        self.signs = rospy.Subscriber('/control/mode', Float64, self.get_mode, queue_size = 1)
        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 1

        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def get_lane_info(self,msg):
        self.lane_vel = msg

    def get_mode(self,msg):
        self.exe_mode = msg.data

    
    def run(self) -> None:
        
        while not rospy.is_shutdown():
            print(self.exe_mode)
            if self.exe_mode == 0: #lane_following
                self.cmd_vel_publisher.publish(self.lane_vel)
            elif self.exe_mode == 1: # construction
                self.lane_vel.linear.x /=2
                self.cmd_vel_publisher.publish(self.lane_vel)
            elif self.exe_mode == 2: # stop
                print("here?")
                t = Twist()
                t.linear.x = 0.000001
                self.cmd_vel_publisher.publish(t)
            rospy.sleep(0.1)




if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

