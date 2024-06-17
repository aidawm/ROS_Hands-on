import rospy
from hw2.srv import *
from hw2.msg import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from math import radians,atan2
import math

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        # self.laser_subscriber = rospy.Subscriber("/closest_distance" , getClosestObj , callback=self.sensor_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.mission_Service_name = "getNexyDestination"
        rospy.wait_for_service(self.mission_Service_name)
        self.count = 0
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") 
        self.angular_speed =0.2 # rad/s

        self.stop_distance = 2 # m
        self.epsilon = 0.01
        self.next_stop_threshold = 3
        self.total_error = 0
        # defining the states of our robot
        self.GO, self.ROTATE , self.GET_OBJ = 0, 1,2
        self.last_getobject = dict() 
        self.state = self.GET_OBJ
         
        
    # checks whether there is an obstacle in front of the robot
    # or not
    def sensor_callback(self, msg):
        # print(msg)

        if msg.distance <= self.stop_distance:
            self.state = self.GET_OBJ

        
    def get_distance (self):
        
        msg = rospy.wait_for_message("/closest_distance" , getClosestObj)

        return msg.distance


    def get_heading(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
  
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw

    def get_pose_info(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
  
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 

        x,y = msg.pose.pose.position.x,msg.pose.pose.position.y
        return x,y,yaw
    
    def get_next_object(self):
        try:
            s = rospy.ServiceProxy(self.mission_Service_name,nextDestination)
            self.next_obj = s()
            print (f"next obj --> {self.next_obj}")
        except rospy.ServiceException as e: 
            print(e)
    
    def calculate_rotation(self,yaw,object_angle):
        rotation = object_angle - yaw
        print(f"i_rotatation = {math.degrees(rotation)}")
        if rotation >180:
            rotation -=360
        elif rotation <= -180:
            rotation +=360

        return rotation
    
    def test(self):
        # for i in range (4):
        #     print ("select obj")
        #     self.get_next_object()
        #     x,y,yaw = self.get_pose_info()
        #     print (f"current yaw={math.degrees(yaw)}")
        #     obj_angle = atan2((self.next_obj.next_y-y),(self.next_obj.next_x-x))
        #     print (f"obj_angle ={math.degrees(obj_angle)}")
        #     rotation = self.calculate_rotation(yaw,obj_angle)
        #     print (f"rotate ={math.degrees(rotation)}") 
        #     print()

        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)
        rospy.sleep(1)

    def run(self):
        
        while not rospy.is_shutdown():
            # self.get_next_object()
            # check whether state is changed or not
            if self.state == self.GET_OBJ:
                if (self.count>=4):
                    break
                # print ("select obj")
                self.get_next_object()
                x,y,yaw = self.get_pose_info()
                # print (f"current yaw={math.degrees(yaw)}")
                # print (f"x = {x}, y={y}\n next x = {self.next_obj.next_x} , next y = {self.next_obj.next_y} ")
                obj_angle = atan2((self.next_obj.next_y-y),(self.next_obj.next_x-x))
                # print (f"obj_angle ={math.degrees(obj_angle)}")
                rotation = self.calculate_rotation(yaw,obj_angle)

                # print (f"rotate ={math.degrees(rotation)}") 
                self.goal_angle = rotation
                self.state = self.ROTATE

                self.last_getobject["x"]= x  
                self.last_getobject["y"]= y 
                self.count+=1
                continue

            
            if self.state == self.GO:
                # print("GO!")
                # print(self.get_heading())
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)

                if (self.get_distance() < self.stop_distance):
                    x,y,yaw = self.get_pose_info()
                    dist = math.sqrt((x-self.last_getobject["x"])**2 + (y-self.last_getobject["y"])**2)
                    if (dist > self.next_stop_threshold): 
                        self.state = self.GET_OBJ
                        self.cmd_publisher.publish(Twist())
                        error =  math.sqrt((x-self.next_obj.next_x)**2 + (y-self.next_obj.next_y)**2)
                        print(f"current state = ({x},{y}), target = ({self.next_obj.next_x},{self.next_obj.next_y}) -----> error = {error}")
                        self.total_error += error
                        rospy.sleep(2)
                continue
            
            self.cmd_publisher.publish(Twist())
            # print ("rotate")
            rospy.sleep(2)
            
            remaining = self.goal_angle
            prev_angle = self.get_heading()
            
            twist = Twist()
            if self.goal_angle > 0 :
                twist.angular.z = self.angular_speed
            else : 
                twist.angular.z = -1*self.angular_speed
            self.cmd_publisher.publish(twist)
            
            # rotation loop
            while abs(remaining) >= self.epsilon:
                # print (remaining)
                current_angle = self.get_heading()
                delta = abs(prev_angle - current_angle)
                if remaining >0:
                    remaining -= delta
                else :
                    remaining += delta
                prev_angle = current_angle
            
            self.cmd_publisher.publish(Twist())

            rospy.sleep(2)
            # print(self.get_heading())
            self.state = self.GO

if __name__=="__main__":
    c = Controller()
    c.run()