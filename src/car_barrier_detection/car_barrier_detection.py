#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import math
from self_driving_turtlebot3.msg import Stop_bar
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

def callback(x):
    pass

def find_distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def find_distance_dot2dot(x1, y1, x2, y2):
    distance = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
    return distance

def find_large_index(arr):
    new_arr = arr[:]
    arr_idx = [0] * len(arr)
    for i in range(len(arr)):
        arr_idx[i] = i

    for i in range(len(arr)):
        for j in range(i+1, len(arr)):
            if arr[i] < arr[j]:
                buffer = arr_idx[j]
                arr_idx[j] = arr_idx[i]
                arr_idx[i] = buffer
                buffer = new_arr[j]
                new_arr[j] = new_arr[i]
                new_arr[i] = buffer
    return arr_idx

def test_linearity(point1, point2, point3):
    threshold_linearity = 50
    x1, y1 = point1
    x2, y2 = point3
    if x2-x1 != 0:
        a = (y2-y1)/(x2-x1)
    else:
        a = 1000
    b = -1
    c = y1 - a*x1
    err = find_distance_dot2line(a, b, c, point2[0], point2[1])

    if err < threshold_linearity:
        return 'yes'
    else:
        return 'no'


def test_distance_equality(point1, point2, point3):

    threshold_distance_equality = 3
    distance1 = find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1])
    distance2 = find_distance_dot2dot(point2[0], point2[1], point3[0], point3[1])
    std = np.std([distance1, distance2])

    if std < threshold_distance_equality:
        return 'yes'
    else:
        return 'no'

class Car_barrier_detection():
    def __init__(self):
        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.track_bar = 'on'
        self.image_showing = 'on'

        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/image_calibrated_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/image_calibrated', Image, self.callback, queue_size=1)

        # poblishers
        self._pub = rospy.Publisher('/stop_bar', Stop_bar, queue_size=1)
        self._sub_3 = rospy.Subscriber('/odom', Odometry, self.callback3, queue_size=1)


        self.stop_bar_count = 0
        self.stop_bar_state = 'go'

        self._cv_bridge = CvBridge()

        self.position_now = None
        self.position_stop_bar = None
        self.state = "detecting"


    def callback(self, image_msg):


        ilowH = 0
        ihighH = 47
        ilowS = 91
        ihighS = 155
        ilowV = 139
        ihighV = 255

        if self.track_bar == 'on':
            cv2.namedWindow('RedFilter')
            # create trackbars for color change
            cv2.createTrackbar('lowH','RedFilter',ilowH,179,callback)
            cv2.createTrackbar('highH','RedFilter',ihighH,179,callback)

            cv2.createTrackbar('lowS','RedFilter',ilowS,255,callback)
            cv2.createTrackbar('highS','RedFilter',ihighS,255,callback)

            cv2.createTrackbar('lowV','RedFilter',ilowV,255,callback)
            #cv2.createTrackbar('highV','RedFilter',ihighV,255,callback)


            # get trackbar positions
            ilowH = cv2.getTrackbarPos('lowH', 'RedFilter')
            ihighH = cv2.getTrackbarPos('highH', 'RedFilter')
            ilowS = cv2.getTrackbarPos('lowS', 'RedFilter')
            ihighS = cv2.getTrackbarPos('highS', 'RedFilter')
            ilowV = cv2.getTrackbarPos('lowV', 'RedFilter')
            #ihighV = cv2.getTrackbarPos('highV', 'RedFilter')

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        frame[50:,:,:] = 0
        # HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
        RedFilter = frame

        RedFilter = cv2.bitwise_and(RedFilter, RedFilter, mask=mask)
        img = np.zeros((481, 640, 3), np.uint8)
        img[1:,:] = RedFilter



        cv_image_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret,cv_image_binary = cv2.threshold(cv_image_gray,1,255,cv2.THRESH_BINARY_INV)

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 700
        params.maxArea = 3000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.9



        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(cv_image_binary)
        frame=cv2.drawKeypoints(frame,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        mean_x = 0.0
        mean_y = 0.0

        # if detected 3 red rectangular
        if len(keypts) == 3:
            for i in range(3):
                mean_x = mean_x + keypts[i].pt[0]/3
                mean_y = mean_y + keypts[i].pt[1]/3
            arr_distances = [0]*3
            for i in range(3):
                arr_distances[i] = find_distance_dot2dot(mean_x, mean_y, keypts[i].pt[0], keypts[i].pt[1])

            # finding thr farthest point from the center
            idx1, idx2, idx3 = find_large_index(arr_distances)
            frame = cv2.line(frame, (int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1])), (int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1])), (255, 0, 0), 5)
            frame = cv2.line(frame, (int(mean_x), int(mean_y)), (int(mean_x), int(mean_y)), (255, 255, 0), 5)
            point1 =  [int(keypts[idx1].pt[0]), int(keypts[idx1].pt[1]-1)]
            point2 = [int(keypts[idx3].pt[0]), int(keypts[idx3].pt[1]-1)]
            point3 = [int(keypts[idx2].pt[0]), int(keypts[idx2].pt[1]-1)]

            # test linearity and distance_equality. If satisfy both one, do next process
            linearity = test_linearity(point1, point2, point3)
            distance_equality = test_distance_equality(point1, point2, point3)
            if linearity == 'yes' or distance_equality == 'yes':
                # finding the anlge of line
                angle = math.atan2(keypts[idx1].pt[1] - keypts[idx2].pt[1], keypts[idx1].pt[0] - keypts[idx2].pt[0]) * 180 / 3.141592
                distance_bar2car = 100 / find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1])
                if angle < 0:
                    angle = angle + 180
                if angle > 90:
                    angle = 180 - angle

                # publishing topic
                if angle < 45:
                    self.stop_bar_count = 40
                    if distance_bar2car > 0.9:
                        self.stop_bar_state = 'slowdown'
                        self.state = "detected"
                    else:
                        self.stop_bar_state = 'stop'

                    print distance_bar2car
                else:
                    self.stop_bar_count = 0
                    self.stop_bar_state = 'go'

                cv2.putText(frame, self.stop_bar_state,(10,300), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2)
                message = Stop_bar()
                message.state = self.stop_bar_state
                message.distance = 100 / find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1])
                message.position1_x = point1[0]
                message.position1_y = point1[1]
                message.position2_x = point3[0]
                message.position2_y = point3[1]
                self._pub.publish(message)

        if self.stop_bar_count > 0:
            self.stop_bar_count -= 1
        if self.stop_bar_count == 1:
            self.stop_bar_state = 'go'
            message = Stop_bar()
            message.state = self.stop_bar_state
            message.distance = 0
            message.position1_x = 0
            message.position1_y = 0
            message.position2_x = 0
            message.position2_y = 0
            self._pub.publish(message)



        # showing image
        if self.image_showing == 'on':
            cv2.imshow("RedFilter", img), cv2.waitKey(1)
            cv2.imshow("detecting stop bar", frame), cv2.waitKey(1)

    def callback3(self, odometry):
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        if self.state == "detected":
            if self.position_stop_bar == None:
                self.position_stop_bar = self.position_now
            if find_distance_dot2dot(self.position_now[0], self.position_now[1], self.position_stop_bar[0], self.position_stop_bar[1]) > 0.2:
                self.stop_bar_count = 80
                self.state = "detecting"
                message = Stop_bar()
                message.state = "stop"
                message.distance = 100 / find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1])
                message.position1_x = 100
                message.position1_y = 100
                message.position2_x = 100
                message.position2_y = 100
                self._pub.publish(message)




    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('car_barrier_detection')
    node = Car_barrier_detection()
    node.main()
