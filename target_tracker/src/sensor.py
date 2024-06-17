import rospy
import tf 
from sensor_msgs.msg import LaserScan
from hw2.msg import *

class Sensor:

    def __init__(self) -> None:
        rospy.init_node("sensor",anonymous=False)
        # rospy.Publisher('motor2_values', motor_value, queue_size=1000)
        print("sensor is on ")
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.pub_scan = rospy.Publisher('closest_distance', getClosestObj, queue_size=1000)
        rospy.spin()


    def laser_callback(self, msg: LaserScan):
        ranges = msg.ranges
        min_value = ranges[0]
        min_deg = 0
        for i in range(len(ranges)):
            if (min_value > ranges[i]):
                min_value = ranges[i]
                min_deg = i
        # print(f"{min_value},{min_deg}")
        self.pub_scan.publish(getClosestObj(min_value))
        


if __name__ == "__main__":
    s = Sensor()

    