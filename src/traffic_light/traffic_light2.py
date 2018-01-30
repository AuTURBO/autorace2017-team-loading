#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import math
from self_driving_turtlebot3.msg import Traffic_light
from cv_bridge import CvBridge, CvBridgeError

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

def test_linearity(point1, point2, point3, point4):
    threshold_linearity = 100
    x1, y1 = point1
    x2, y2 = point4
    a = (y2-y1)/(x2-x1)
    b = -1
    c = y1 - a*x1
    err_1 = find_distance_dot2line(a, b, c, point2[0], point2[1])
    err_2 = find_distance_dot2line(a, b, c, point3[0], point3[1])
    err_total = err_1 + err_2

    if err_total < threshold_linearity:
        return 'yes'
    else:
        return 'no'


def test_distance_equality(point1, point2, point3, point4):

    threshold_distance_equality = 3
    distance1 = min(find_distance_dot2dot(point1[0], point1[1], point2[0], point2[1]), find_distance_dot2dot(point1[0], point1[1], point3[0], point3[1]))
    distance2 = find_distance_dot2dot(point2[0], point2[1], point3[0], point3[1])
    distance3 = min(find_distance_dot2dot(point2[0], point2[1], point4[0], point4[1]), find_distance_dot2dot(point3[0], point3[1], point4[0], point4[1]))
    std = np.std([distance1, distance2, distance3])

    if std < threshold_distance_equality:
        return 'no'
    else:
        return 'no'

class Traffic_light_detection():
    def __init__(self):
        self._cv_bridge = CvBridge()

        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/image_calibrated_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/image_calibrated', Image, self.callback, queue_size=1)

        self._pub = rospy.Publisher('/traffic_light', Traffic_light, queue_size=1)

        self.mode = 'running'   # tuning, running
        self.showing_image = 'no'
        self.detecting_color = 'red'  # red, orange, green

        #self.filter_color = {'red':(0, 20, 60, 190, 93, 255), 'green':(69, 110, 62, 165, 187, 255), 'orange':(255, 255, 255, 207, 100, 255)}
        self.filter_color = {'red':(0, 22, 85, 198, 172, 255), 'green':(69, 100, 96, 198, 172, 255), 'orange':(255, 255, 255, 207, 100, 255)}
        self.filter_tuning = (170, 192, 81, 128, 120, 255)
        self.black_color_box_size = {'red':{'w':20, 'top':30, 'bottom':100}, 'green':{'w':10, 'top':40, 'bottom':10}, 'orange':{'w':20, 'top':90, 'bottom':40}}

        self.orange_count = 0
        self.red_count = 0

        self.traffic_light_end = "no"
        self.state = "run"

    def callback(self, image_msg):
        if self.state == "end":
            returnr


        if self.mode == 'tuning':
            cv2.namedWindow('ColorFilter')

            ilowH, ihighH, ilowS, ihighS, ilowV, ihighV = self.filter_tuning

            # create trackbars for color change
            cv2.createTrackbar('lowH','ColorFilter',ilowH,255,callback)
            cv2.createTrackbar('highH','ColorFilter',ihighH,255,callback)

            cv2.createTrackbar('lowS','ColorFilter',ilowS,255,callback)
            cv2.createTrackbar('highS','ColorFilter',ihighS,255,callback)

            cv2.createTrackbar('lowV','ColorFilter',ilowV,255,callback)
            #cv2.createTrackbar('highV','image',ihighV,255,callback)


            # get trackbar positions
            ilowH = cv2.getTrackbarPos('lowH', 'ColorFilter')
            ihighH = cv2.getTrackbarPos('highH', 'ColorFilter')
            ilowS = cv2.getTrackbarPos('lowS', 'ColorFilter')
            ihighS = cv2.getTrackbarPos('highS', 'ColorFilter')
            ilowV = cv2.getTrackbarPos('lowV', 'ColorFilter')
            #ihighV = cv2.getTrackbarPos('highV', 'ColorFilter')

            self.filter_tuning = ilowH, ihighH, ilowS, ihighS, ilowV, ihighV

        else:
            ilowH, ihighH, ilowS, ihighS, ilowV, ihighV = self.filter_color[self.detecting_color]

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
        ColorFilter = frame

        ColorFilter = cv2.bitwise_and(ColorFilter, ColorFilter, mask=mask)

        cv_image_gray = cv2.cvtColor(ColorFilter, cv2.COLOR_RGB2GRAY)
        ret,cv_image_binary = cv2.threshold(cv_image_gray,1,255,cv2.THRESH_BINARY_INV)
        # cv2.imshow('image', cv_image_binary), cv2.waitKey(1)

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 800

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.6

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6


        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(cv_image_binary)
        frame=cv2.drawKeypoints(frame,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        mean_x = 0.0
        mean_y = 0.0


        col1 = 300
        col2 = 640

        low1 = 0
        low2 = 200
        frame = cv2.line(frame, (col1, low1), (col1, low2), (255, 255, 0), 5)
        frame = cv2.line(frame, (col1, low2), (col2, low2), (255, 255, 0), 5)
        frame = cv2.line(frame, (col2, low2), (col2, low1), (255, 255, 0), 5)
        frame = cv2.line(frame, (col2, low1), (col1, low1), (255, 255, 0), 5)

        # if detected more than 1 red light
        for i in range(len(keypts)):
            point_col = int(keypts[i].pt[0])
            point_low = int(keypts[i].pt[1])
            print 'detected'


            if point_col > col1 and point_low < low2:
                print 'yes'
                if self.detecting_color == 'green':
                    if self.red_count > 0:
                        self.red_count = 0
                        self.state = "end"
                        message = Traffic_light()
                        message.color = "fast"
                        message.position_x = point_col
                        message.position_y = point_low
                        self._pub.publish(message)
                    else:
                        self.red_count = 0
                        message = Traffic_light()
                        message.color = "green"
                        message.position_x = point_col
                        message.position_y = point_low
                        self._pub.publish(message)

                if self.detecting_color == 'red':
                    self.red_count = 30
                    message = Traffic_light()
                    message.color = self.detecting_color
                    message.position_x = point_col
                    message.position_y = point_low
                    self._pub.publish(message)

        if self.red_count == 1:
            message = Traffic_light()
            message.color = "green"
            message.position_x = 100
            message.position_y = 100
            self._pub.publish(message)


        if self.red_count > 0:
            self.red_count -= 1

        if self.detecting_color == 'red':
            self.detecting_color = 'green'
        elif self.detecting_color == 'green':
            self.detecting_color = 'red'

        # showing image
        if self.showing_image == 'yes':
            cv2.imshow("ColorFilter", ColorFilter), cv2.waitKey(1)
            cv2.imshow("detecting stop bar", frame), cv2.waitKey(1)
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light2')
    node = Traffic_light_detection()
    node.main()
