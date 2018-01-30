#!/usr/bin/env python
import math
from std_msgs.msg import String
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

def callback(x):
    pass

def sign(num):
    if num > 0:
        return 1
    else:
        return -1

def distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    sign_ = sign(x0*a + y0*b + c) * sign(a)
    return sign_, distance

def distance_dot2dot(point1, point2):
    return math.sqrt((point2[1] - point1[1]) * (point2[1] - point1[1]) + (point2[0] - point1[0]) * (point2[0] - point1[0]))

def centroid(arr, low_point, col_start, col_end):
    count = 0
    center_low = 0
    center_col = 0

    for i in range(col_start,col_end):
        if arr[low_point][i] == 255:
            center_col = center_col + i
            count = count + 1
        i += 4

    center_low = low_point
    center_col = center_col / count
    return center_low, center_col



class Lane_tracer():
    def __init__(self):

        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.image_show = 'off'  # monitering image on off

        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub_1 = rospy.Subscriber('/image_birdeye_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub_1 = rospy.Subscriber('/image_birdeye', Image, self.callback, queue_size=1)
        self._sub_2 = rospy.Subscriber('/command_lane_follower', String, self.receiver_from_core, queue_size=1)
        self._sub_3 = rospy.Subscriber('/odom', Odometry, self.callback3, queue_size=1)
        # publishers
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self._pub_2 = rospy.Publisher('/command_maze', String, queue_size=1)
        self._cv_bridge = CvBridge()

        self.run = 'yes'

        self.lane_position = 0
        self.lane_existance = 'yes'
        self.stop_count = 0
        self.speed = 2 # Increasing this variable makes angular and linear speed fast in same ratio
        self.fast = 'off'
        self.count = 0
        self.state = "usual" #usual, turn_go, turn_stop
        self.count2 = 0
        self.count3 = 0

        self.position_now = None
        self.position_stop = None


    def PIDcontrol(self, x0, y0, x1, y1, x2, y2):
        Ip = 0.2
        distance = 350
        if x1 != x2:
            a = (float(y2)-float(y1))/(float(x2)-float(x1))
        else:
            a = (float(y2)-float(y1))/1
        b = -1
        c = -a*x1 + y1
        theta_current = -math.atan(1/a)*180/np.pi
        sign_, err = distance_dot2line(a, b, c, x0, y0)
        theta_desired = (err - distance) * Ip * sign_
        theta_wheel = (theta_desired - theta_current) * 0.005 * self.speed # origin : 0.005 next : 0.007
        return sign_, theta_wheel

    def callback(self, image_msg):
        print self.count
        self.count += 1

        if self.count > 50:
            self.speed = 0.3

        if self.count > 250:
            self.speed = 2
            self.fast = "on"

        if self.count == 1200:
            self.state = "turn_stop"
            self.stop_count = 0

        if self.state == "turn_stop":
            self.publishing_vel(0,0,0,0,0,0)
            self.run = "stop"
            self.stop_count += 1

        if self.stop_count > 40:
            self.state = "turn_go"
            self.run = "yes"

        if self.run == 'stop':
            return


        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        else:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "mono8")

        # setting desired path line
        low_point1 = 599

        while(1):
            try:
                _, col_point1 =  centroid(cv_image, low_point1, 300, cv_image.shape[1])
                break
            except Exception as e:
                if low_point1 < 200:
                    self.lane_existance = 'no'
                    break
                else:
                    low_point1 -= 50

        low_point2 = low_point1 - 50
        while (1):
            try:
                _, col_point2 = centroid(cv_image, low_point2, 300, cv_image.shape[1])
                self.lane_existance = 'yes'
                break
            except Exception as e:
                if low_point2 < 150:
                    self.lane_existance = 'no'
                    break
                else:
                    low_point2 -= 50

        if self.state == "maze":
            self.count2 += 1
            if self.lane_existance == 'yes':
                if self.count2 > 100:
                    self.count3 += 1
                    if self.count3 > 10:
                        self.state = "usual"
                        self.run = "yes"
                        self._pub_2.publish("maze_end")


        if self.state == "turn_go":
            if self.lane_existance == 'no':
                self._pub_2.publish("maze_start")
                self.state = "maze"


        if self.state == "maze":
            return


        # when there is no lane detected turtlebot3 will turn right with 0.2 angular velocity
        if self.lane_existance == 'no':
            print "no lane"
            # showing binary and original images
            if self.image_show == 'on':
                cv2.imshow('tracer', cv_image), cv2.waitKey(1)

            angular_z = self.lane_position * 0.2 * self.speed
            linear_x = 0.06 * self.speed
            self.publishing_vel(0, 0, angular_z, linear_x, 0, 0)
            return

        # drawing desired path using point1, point2
        cv_image = cv2.line(cv_image, (col_point1, low_point1), (col_point2, low_point2), (0, 0, 0), 3)
        cv_image = cv2.line(cv_image, (col_point1, low_point1), (col_point2, low_point2), (255, 255, 255), 1)

        # setting and drawing current position
        low_position = (low_point1 + low_point2)/2
        col_position = 500
        cv_image = cv2.circle(cv_image, (col_position, low_position), 3, (255, 255, 255), thickness=5, lineType=8, shift=0)
        cv_image = cv2.circle(cv_image, (col_position, low_position), 3, (0, 0, 0), thickness=3, lineType=8, shift=0)


        if self.image_show == 'on':
            # showing image
            cv2.imshow('tracer', cv_image), cv2.waitKey(1)

        # setting cmd_vel using PID control function
        self.lane_position, angular_z = self.PIDcontrol(col_position, low_position, col_point1, low_point1, col_point2, low_point2)

        # poblishing cmd_vel topic
        self.publishing_vel(0, 0, angular_z, 0.06 * self.speed, 0, 0)


    def callback3(self, odometry):
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        if self.state == "turn_go":
            if self.position_stop == None:
                self.position_stop = self.position_now
            if distance_dot2dot(self.position_now, self.position_stop) > 0.2:
                self.position_stop = None
                self.state = "turn_stop"
                self.publishing_vel(0,0,0,0,0,0)
                self.run = "stop"
                self.stop_count = 0

    def receiver_from_core(self, command):
        self.run = command.data
        if self.run == 'fast':
            self.fast = 'on'
        if self.run == 'go':
            self.speed = 0.3

        if self.fast == 'on':
            self.speed = 2

        if self.run == 'slowdown':
            self.speed = 0.3

        if self.run == 'stop':
            self.publishing_vel(0, 0, 0, 0, 0, 0)

    def publishing_vel(self, angular_x, angular_y, angular_z, linear_x, linear_y, linear_z):
        vel = Twist()
        vel.angular.x = angular_x
        vel.angular.y = angular_y
        vel.angular.z = angular_z
        vel.linear.x = linear_x
        vel.linear.y = linear_y
        vel.linear.z = linear_z
        self._pub.publish(vel)


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    node = Lane_tracer()
    node.main()
