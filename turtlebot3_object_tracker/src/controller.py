#!/usr/bin/python3

import math

# ROS
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import Detection, DetectionRequest


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 1

        # TODO: Create a service proxy for your human detection service
        rospy.wait_for_service('detection_service')
        self.detection_service_proxy = rospy.ServiceProxy('detection_service', Detection)

        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=1)


    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot
                detection_service_response = self.get_detection_info()
                if detection_service_response.detection_info:
                    self.publish_robot_movement(detection_service_response)
                else:
                    self.cmd_vel_publisher.publish(self.freeze)
                    rospy.loginfo("freeze")

                rospy.sleep(0.1)


        except rospy.exceptions.ROSInterruptException:
            pass
    
                
    def get_detection_info(self):
        try:
        
            request = DetectionRequest()
            request.label = "person"

            response = self.detection_service_proxy(request)

            return response

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % str(e))

    def publish_robot_movement (self, detection_service_response):
    
        error = self.calculate_angular_error(detection_service_response)
        angular_vel = self.angular_vel_coef * error
        self.move.angular.z = angular_vel

        self.cmd_vel_publisher.publish(self.move)

    def calculate_angular_error(self, detection_service_response):
    
        upper_left = detection_service_response.detection_info[0]
        lower_right = detection_service_response.detection_info[1]

        image_size = detection_service_response.image_size
        image_width = image_size[1]

        center_x = (upper_left.x + lower_right.x) / 2

        desired_position = image_width / 2

        # Calculate the error as the difference between the detected position and the desired position
        error = desired_position - center_x

    
        angle = math.atan2(error, image_width) 

        return angle

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

