#!/usr/bin/env python
import math
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from self_driving_turtlebot3.msg import Traffic_light
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from self_driving_turtlebot3.msg import Stop_bar
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import os
import threading

def converting_to_maze_mode():
    os.system("rosnode kill calibration_and_birdeyeview")
    os.system("rosnode kill car_barrier_detection")
    os.system("rosnode kill lane_follower")
    os.system("rosnode kill parking")
    os.system("rosnode kill traffic_light2")
    os.system("rosnode kill usb_cam")
    thread = threading.Thread(target=os.system, args=('roslaunch self_driving_turtlebot3 mode_maze.launch',))
    thread.setDaemon(True)
    thread.start()

def converting_to_tracer_mode():
    os.system("rosnode kill maze_pathfinder")
    os.system("rosnode kill turtlebot3_slam_gmapping")
    thread = threading.Thread(target=os.system, args=('roslaunch self_driving_turtlebot3 mode_tracer.launch',))
    thread.setDaemon(True)
    thread.start()

class Core():
    def __init__(self):

        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.image_show = 'no' # no ,yes

        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub_1 = rospy.Subscriber('/image_calibrated_compressed', CompressedImage, self.monitoring, queue_size=1)
        else:
            self._sub_1 = rospy.Subscriber('/image_calibrated', Image, self.monitoring, queue_size=1)
            self._sub_1 = rospy.Subscriber('/usb_cam/image_raw', Image, self.maze_check, queue_size=1)

        self._sub_2 = rospy.Subscriber('/stop_bar', Stop_bar, self.receiver_stop_bar, queue_size=1)
        self._sub_3 = rospy.Subscriber('/traffic_light', Traffic_light, self.receiver_traffic_light, queue_size=1)
        self._sub_4 = rospy.Subscriber('/parking', String, self.receiver_parking, queue_size=1)
        self._sub_5 = rospy.Subscriber('/scan', LaserScan, self.callback2, queue_size=1)
        self._sub_6 = rospy.Subscriber('/maze', String, self.receiver_maze, queue_size=1)
        self._sub_7 = rospy.Subscriber('/signal_sign', String, self.receiver_signal_sign, queue_size=1)
        self._sub_8 = rospy.Subscriber('/objects', Float32MultiArray, self.receiver_object_find, queue_size=1)
        self._pub_1 = rospy.Publisher('/command_lane_follower', String, queue_size=1)
        self._pub_2 = rospy.Publisher('/command_maze', String, queue_size=1)

        self._cv_bridge = CvBridge()

        self.image = None
        self.mode = 'lane_follower' # lane_follower, parking, maze_solver
        self.state = 'go' # stop, go, slowdown : this is only used when lane_follower mode
        self.state_maze = 'outside' # inside, outside

        self.traffic_light_color = 'green'
        self.traffic_light_detected = 'no'
        self.traffic_light_x = 0
        self.traffic_light_y = 0

        self.stop_bar_state = 'go'
        self.stop_bar_distance = None
        self.stop_bar_detected = 'no'
        self.stop_bar_point1_x = 0
        self.stop_bar_point1_y = 0
        self.stop_bar_point2_x = 0
        self.stop_bar_point2_y = 0


        self.parking = None

        self.maze = None

        self.signal_sign = None

        self.find_object = None

        self.wall_detected = None
        self.count = 0

    def maze_check(self, image_msg):

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.wall_detected == "yes":
            mean = np.mean(self.image)
            print mean
            if mean < 200 and self.maze != "maze_start":
                self.maze = "maze_start"
                self.commander()
                self._pub_2.publish(self.maze)


    def monitoring(self, image_msg):

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        print self.wall_detected

        if self.wall_detected == "yes":
            mean = np.mean(self.image)
            print mean
            if mean < 30:
                self.maze = "maze_start"
                self.commander()


        if self.traffic_light_detected == 'yes':
            #if self.image_show == 'yes':
                #self.draw_traffic_light()
            self.traffic_light_detected = 'no'

        if self.stop_bar_detected == 'yes':
            if self.image_show == 'yes':
                self.draw_stop_bar()
            self.stop_bar_detected = 'no'

        #cv2.putText(self.image, self.state, (100, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if self.image_show == 'yes':
            cv2.imshow("monitoring", self.image), cv2.waitKey(1)

    def commander(self):

        if self.stop_bar_state == 'stop' or self.parking == 'parking_lot_detected' or (self.traffic_light_color == 'red' and self.traffic_light_x > 550) or self.maze == "maze_start":
            self.state = 'stop'

        elif self.stop_bar_state == 'slowdown' or self.signal_sign == 'WARNING' or self.find_object > 0:
            self.state = 'slowdown'
        elif self.traffic_light_color == 'fast':
            self.state = 'fast'
        else:
            self.state = 'go'
        self._pub_1.publish(self.state)

    def receiver_stop_bar(self, stop_bar):
        self.stop_bar_detected = 'yes'
        self.stop_bar_distance = stop_bar.distance
        self.stop_bar_state = stop_bar.state
        self.stop_bar_point1_x = stop_bar.position1_x
        self.stop_bar_point1_y = stop_bar.position1_y
        self.stop_bar_point2_x = stop_bar.position2_x
        self.stop_bar_point2_y = stop_bar.position2_y
        self.commander()

    def receiver_traffic_light(self, traffic_light):
        self.traffic_light_color = traffic_light.color
        self.traffic_light_x = traffic_light.position_x
        self.traffic_light_y = traffic_light.position_y
        self.traffic_light_detected = 'yes'
        self.commander()
    def receiver_parking(self, parking):
        self.parking = parking.data
        self.commander()

    def receiver_maze(self, maze):
        self.maze = maze.data
        if self.maze == "maze_end":
            self.commander()

    def receiver_signal_sign(self, signal_sign):
        self.signal_sign = signal_sign.data
        self.commander()

    def receiver_object_find(self, find_object):
        if len(find_object.data) > 0:
            self.find_object = 10
            print 'yes'
        elif self.find_object > 0:
                self.find_object -= 1
        print self.find_object
        self.commander()

    def draw_traffic_light(self):
        self.image = cv2.circle(self.image, (self.traffic_light_x, self.traffic_light_y), 10, (0, 0, 255), thickness=3, lineType=8, shift=0)
        if self.traffic_light_color == 'red':
            cv2.putText(self.image, self.traffic_light_color, (self.traffic_light_x + self.traffic_light_w/3, self.traffic_light_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if self.traffic_light_color == 'green':
            cv2.putText(self.image, self.traffic_light_color, (self.traffic_light_x + self.traffic_light_w/3, self.traffic_light_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def draw_stop_bar(self):
        cv2.line(self.image, (self.stop_bar_point1_x, self.stop_bar_point1_y), (self.stop_bar_point2_x, self.stop_bar_point2_y), (255, 255, 0), 5)
        cv2.putText(self.image, str(self.stop_bar_distance), ((self.stop_bar_point1_x + self.stop_bar_point2_x) / 2, (self.stop_bar_point1_y + self.stop_bar_point2_y)/2 - 50),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def main(self):
        rospy.spin()

    def callback2(self, scan):
        return
        scan_arr = np.zeros((180), np.float16)
        for i in range(0, 90):
            scan_arr[i] = scan.ranges[i]*math.sin((i))
        count_between_distances = 0
        distance1 = 0.1
        distance2 = 0.4
        for i in range(180):
            if scan_arr[i] > distance1 and scan_arr[i] < distance2:
                count_between_distances += 1
        print count_between_distances
        if count_between_distances > 13:
            self.wall_detected = "yes"
        else:
            self.wall_detected = "no"


if __name__ == '__main__':
    rospy.init_node('core')
    node = Core()
    node.main()
