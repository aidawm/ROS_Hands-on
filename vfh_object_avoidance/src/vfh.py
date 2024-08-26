#!/usr/bin/python3

import math
import numpy as np
import rospy
import tf
from math import radians, pi
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

Straight = 0
Rotate = 1

class Controller():
    def __init__(self, *args, **kwargs):
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.targets = [
            (4.2, 2),
            (4.2, 3.5),
            (3.5, 4.5),
            (2, 1.2),
            (0.5, 1.5),
            (7, 4),
            (13, 7)
        ]

        self.current_target_index = 0

        self.init_x = 0
        self.init_y = 0

        self.a = 1
        self.b = 0.25
        self.l = 2

        self.angular_speed = 0.2
        self.linear_speed = 0.15

        self.alpha = radians(5)
        self.angle_max = 2 * pi
        self.k = int(self.angle_max / self.alpha) + 1

        self.smax = 10

        self.threshold = 6
        self.epsilon = 0.3

        self.sector = []
        self.normalized = []
        self.h = []

        rate = 10
        self.r = rospy.Rate(rate)

        self.state = Straight

        self.goal_x, self.goal_y = self.targets[self.current_target_index]
        self.last_target_update_time = rospy.get_time()

    def get_position(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        return msg.pose.pose.position

    def get_obstacle(self):
        msg = rospy.wait_for_message("/scan", LaserScan)
        return msg

    def angular_distance(self):
        pose = self.get_position()
        angle = math.atan2(self.goal_y - pose.y, self.goal_x - pose.x)
        return angle

    def get_heading(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return yaw

    def sectorize(self):
        s = 0
        e = 5
        c = 0
        counter = 0
        self.sector = []
        obstacle_prob = self.get_obstacle()
        while e <= 360 and s < 355:
            self.sector.append([0 for _ in range(5)])
            for i in obstacle_prob.ranges[s:e]:
                x = self.a - (self.b * i)
                if x < 0:
                    x = 0
                self.sector[counter][c] = x
                c += 1

            e += 5
            s += 5
            c = 0
            counter += 1

    def density(self):
        self.h = [sum(x) for x in self.sector]

    def normalizer(self):
        self.normalized = []
        for i, _ in enumerate(self.h):
            if i == 0:
                self.normalized.append(((2 * self.h[i]) + self.h[i + 1] + self.h[-1]) / 4)
                continue
            if i == len(self.h) - 1:
                self.normalized.append(((2 * self.h[i]) + self.h[0] + self.h[i - 1]) / 4)
                continue
            self.normalized.append(((2 * self.h[i]) + self.h[i + 1] + self.h[i - 1]) / 4)

    def euclidean_distance(self):
        pose = self.get_position()
        return math.sqrt((pose.x - self.goal_x) ** 2 + (pose.y - self.goal_y) ** 2)

    def normalize_angle(self, angle):
        res = angle
        while res > 2 * pi:
            res -= 2.0 * pi
        while res < -2 * pi:
            res += 2.0 * pi
        return res

    def update_target(self):
        self.current_target_index = self.current_target_index + 1
        self.goal_x, self.goal_y = self.targets[self.current_target_index]
        self.last_target_update_time = rospy.get_time()

    def planning(self):
        self.r.sleep()

        robot_heading = self.get_heading()
        if robot_heading < 0:
            robot_heading = robot_heading + (2 * pi)

        robot_heading_sector = int(robot_heading / self.alpha)
        rotation = self.get_heading()
        pose = self.get_position()
        angle = math.atan2(self.goal_y - pose.y, self.goal_x - pose.x)
        if angle < 0:
            angle = angle + (2 * pi)
        ktarget = int  (angle / self.alpha) + 1

        kn, kf = None, None
        i = ktarget - 1
        j = ktarget + 1

        goal_sector = None

        self.normalized = np.asarray(self.normalized)
        self.normalized = np.array(self.normalized.tolist() + self.normalized.tolist())

        valleys = (self.normalized <= self.threshold)

        if (not valleys[ktarget]) and (ktarget <= len(self.normalized)):
            while i >= 0 or j < len(self.normalized):
                res = []
                if i >= 0 and valleys[i]:
                    ind = i
                    kn = i
                    valley_size = 1
                    while ind >= 0 and valleys[ind]:
                        valley_size += 1
                        ind -= 1
                        if valley_size >= self.smax:
                            kf = kn - self.smax
                        else:
                            kf = ind + 1

                    if kf is None :
                        kf = 0 
                    goal_sector = (kf + kn) // 2
                    res.append((kf, kn, ktarget, goal_sector))

                i -= 1

                if j < len(self.normalized) and valleys[j]:
                    ind = j
                    kn = j
                    valley_size = 1
                    while ind < len(self.normalized) and valleys[ind]:
                        valley_size += 1
                        ind += 1
                        if valley_size >= self.smax:
                            kf = kn + self.smax
                        else:
                            kf = ind - 1

                    if kf is None :
                        kf = 0 
                    goal_sector = (kf + kn) // 2
                    res.append((kf, kn, ktarget, goal_sector))

                j += 1

                if len(res) == 2:
                    (kf, kn, ktarget, goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]
                    break
                elif len(res) == 1:
                    (kf, kn, ktarget, goal_sector) = res[0]
                    break

        else:
            res = []
            ind = i
            kn = i
            valley_size = 1
            while ind >= 0 and valleys[ind]:
                valley_size += 1
                ind -= 1
                if valley_size >= self.smax:
                    kf = kn - self.smax
                else:
                    kf = ind + 1

            if kf is None :
                kf = 0 
            goal_sector = (kf + kn) // 2
            res.append((kf, kn, ktarget, goal_sector))

            ind = j
            kn = j
            valley_size = 1
            while ind < len(self.normalized) and valleys[ind]:
                valley_size += 1
                ind += 1
                if valley_size >= self.smax:
                    kf = kn + self.smax
                else:
                    kf = ind - 1

            if kf is None :
                kf = 0 
            goal_sector = (kf + kn) // 2
            res.append((kf, kn, ktarget, goal_sector))
            (kf, kn, ktarget, goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]

        goal_angle = self.normalize_angle(goal_sector * self.alpha)
        tw_msg = Twist()
        last_angle = rotation
        first_angle = rotation
        turn_angle = 0
        tw_msg.angular.z = self.angular_speed * (goal_angle - rotation) / abs(goal_angle - rotation)

        while abs(turn_angle) < abs(first_angle - goal_angle) and not rospy.is_shutdown():
            self.cmd_publisher.publish(tw_msg)
            self.r.sleep()
            rotation = self.get_heading()
            delta_angle = self.normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation

        ms = Twist()
        self.cmd_publisher.publish(ms)
        ms = Twist()
        ms.linear.x = self.linear_speed / 2
        self.cmd_publisher.publish(ms)
        self.state = Straight

    def run(self):
        while not rospy.is_shutdown():
            msg = self.get_obstacle()
            fronts = msg.ranges[0:10]
            fronts += msg.ranges[350:359]
            front = min(fronts)
            goal_d = self.euclidean_distance()

            if goal_d <= 1:
                    if front < self.epsilon + 0.2:
                        rospy.sleep(1)
                        self.update_target()
                        print("Changing the target..")
                        self.planning()
                        self.goal_x, self.goal_y = self.targets[self.current_target_index]
                    else:
                        self.update_target()
                        print("Changing the target..")
                        self.goal_x, self.goal_y = self.targets[self.current_target_index]
                        tw_msg = Twist()
                        angle = math.atan2(self.goal_y - pose.y, self.goal_x - pose.x)
                        rotation = self.get_heading()
                        tw_msg.angular.z = self.angular_speed * 2 * (angle-rotation) / (abs(angle-rotation))
                        self.cmd_publisher.publish(tw_msg) 
                        tw_msg = Twist()          
                        tw_msg.linear.x = self.linear_speed
                        self.cmd_publisher.publish(tw_msg)

            else:
                if goal_d > 1:
                    if front < self.epsilon + 0.2:
                        tw_msg = Twist()
                        self.cmd_publisher.publish(tw_msg)
                        self.sectorize()
                        self.density()
                        self.normalizer()
                        self.planning()
                        rospy.loginfo("Avoiding obstacle...")
                    else:
                        tw_msg = Twist()
                        tw_msg.linear.x = self.linear_speed
                        self.cmd_publisher.publish(tw_msg)
                else:
                    rospy.sleep(1)

            pose = self.get_position()
            self.init_x = pose.x
            self.init_y = pose.y

        rospy.spin()

if __name__ == "__main__":
    c = Controller()
    c.run()