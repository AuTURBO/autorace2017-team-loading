#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import tensorflow as tf
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

def fully_connected_neural_network(X, keep_prob):
    W1 = tf.get_variable("W1", shape=[784, 512], initializer=tf.contrib.layers.xavier_initializer())
    b1 = tf.Variable(tf.random_normal([512]))
    L1 = tf.nn.relu(tf.matmul(X, W1) + b1)
    L1 = tf.nn.dropout(L1, keep_prob=keep_prob)

    W2 = tf.get_variable("W2", shape=[512, 512], initializer=tf.contrib.layers.xavier_initializer())
    b2 = tf.Variable(tf.random_normal([512]))
    L2 = tf.nn.relu(tf.matmul(L1, W2) + b2)
    L2 = tf.nn.dropout(L2, keep_prob=keep_prob)

    W3 = tf.get_variable("W3", shape=[512, 512], initializer=tf.contrib.layers.xavier_initializer())
    b3 = tf.Variable(tf.random_normal([512]))
    L3 = tf.nn.relu(tf.matmul(L2, W3) + b3)
    L3 = tf.nn.dropout(L3, keep_prob=keep_prob)

    W4 = tf.get_variable("W4", shape=[512, 512], initializer=tf.contrib.layers.xavier_initializer())
    b4 = tf.Variable(tf.random_normal([512]))
    L4 = tf.nn.relu(tf.matmul(L3, W4) + b4)
    L4 = tf.nn.dropout(L4, keep_prob=keep_prob)

    W5 = tf.get_variable("W5", shape=[512, 10], initializer=tf.contrib.layers.xavier_initializer())
    b5 = tf.Variable(tf.random_normal([10]))
    hypothesis = tf.nn.softmax(tf.matmul(L4, W5) + b5)
    return hypothesis


def center(points):
    center_x = (points[0][0][0] + points[1][0][0] + points[2][0][0] + points[3][0][0])/4.0
    center_y = (points[0][0][1] + points[1][0][1] + points[2][0][1] + points[3][0][1])/4.0
    return center_x, center_y


def find_position(points):
    center_x, center_y = center(points)
    index = np.zeros(4)
    existance0 = 'no'
    existance1 = 'no'
    existance2 = 'no'
    existance3 = 'no'
    existanceall = 'no'
    for i in range(4):
        if points[i][0][0] < center_x:
            if points[i][0][1] > center_y:
                index[3] = i
                existance3 = 'yes'
            else:
                index[0] = i
                existance0 = 'yes'
        else:
            if points[i][0][1] > center_y:
                index[2] = i
                existance2 = 'yes'
            else:
                index[1] = i
                existance1 = 'yes'

    if existance0 == 'yes' and existance1 == 'yes' and existance2 == 'yes' and existance3 == 'yes':
        existanceall = 'yes'
    return existanceall, index


def find_angle(point1, point0, point2):
    y1 = point1[1] - point0[1]
    y2 = point2[1] - point0[1]
    x1 = point1[0] - point0[0]
    x2 = point2[0] - point0[0]
    angle = math.atan2(y1*x2 - x1*y2, x1*x2+y1*y2)*180/np.pi
    return abs(angle)

def distinguish_rectangular(screenCnt):
    threshold_angle = 20
    existance, index = find_position(screenCnt)
    for i in range(4):
        if find_angle(screenCnt[(i+0)%4][0], screenCnt[(i+1)%4][0], screenCnt[(i+2)%4][0]) > 90 + threshold_angle or find_angle(screenCnt[(i+0)%4][0], screenCnt[(i+1)%4][0], screenCnt[(i+2)%4][0]) < 90 - threshold_angle:
            satisfaction_angle = 'no'
            break
        satisfaction_angle = 'yes'
    if satisfaction_angle == 'yes' and existance == 'yes':
        return 'yes'


class TrafficSignDetection():
    def __init__(self):
        self.X = tf.placeholder(tf.float32, [None, 784])
        self.keep_prob = tf.placeholder("float")
        self.Y = fully_connected_neural_network(self.X, self.keep_prob)
        self.sess = tf.InteractiveSession()
        init_op = tf.global_variables_initializer()
        self.sess.run(init_op)
        self.saver = tf.train.Saver()
        self.saver.restore(self.sess, "/home/kihoon/catkin_ws/src/self_driving_turtlebot3/src/signal_sign_detection/model/model.ckpt")

        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/image_calibrated_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/image_calibrated', Image, self.callback, queue_size=1)

        # publishers
        self._pub = rospy.Publisher('/signal_sign', String, queue_size=1)

        self._cv_bridge = CvBridge()
        self.softmax_threshold = 0.9

    def callback(self, image_msg):
        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # save original image
        image_origin = np.copy(image)
        # converting to gray
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # adding blur
        image_smoothing = cv2.bilateralFilter(image_gray, 5, 5, 5)
        # findign edge
        image_edged = cv2.Canny(image_smoothing, 20, 100)
        # making egde to be thicker
        kernel = np.ones((5,5),np.uint8)
        image_dilation = cv2.dilate(image_edged,kernel,iterations = 1)
        #finding contours
        _, cnts, hierarchy = cv2.findContours(image_dilation.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
        screenCnt = None
        area_pre = 100000


        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # do this process if contour have 4 edge
            if len(approx) == 4:
                screenCnt = approx
                area_now = cv2.contourArea(c)
                check_rectangular = distinguish_rectangular(screenCnt)
                # do this process if all angles of rectangular between 70 and 110
                if check_rectangular == 'yes' and area_pre - area_now < 10000:
                    cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3) # drawing rectangular box
                    for j in range(4):  # drawing circles in vertex
                        image = cv2.circle(image, (screenCnt[j][0][0], screenCnt[j][0][1]), 2, (0, 0, 255), thickness=3, lineType=8, shift=0)
                    _, index = find_position(screenCnt)
                    self.object_recognition(screenCnt[int(index[0])][0], screenCnt[int(index[1])][0], screenCnt[int(index[2])][0], screenCnt[int(index[3])][0], image_origin)
                area_pre = area_now
                screenCnt_pre = screenCnt
        # showing images
        cv2.imshow("image_edged", image_edged), cv2.waitKey(1)
        cv2.imshow("image_dilation", image_dilation), cv2.waitKey(1)
        cv2.imshow("image", image), cv2.waitKey(1)

    def object_recognition(self, point1, point2, point3, point4, image_origin):
        pts_src = np.array([point1, point2, point3, point4])
        # Homography processing to make flat image
        pts_dst = np.array([[0, 0], [149, 0], [149, 149], [0, 149]])
        h, status = cv2.findHomography(pts_src, pts_dst)
        cv_Homography = cv2.warpPerspective(image_origin, h, (150, 150))
        # showing flat image
        cv2.imshow("cv_Homography", cv_Homography), cv2.waitKey(1)
        # resize and convert to numpy arrray to feed neural network
        image_resize = cv2.resize(cv_Homography, (28, 28))
        image_gray = cv2.cvtColor(image_resize, cv2.COLOR_BGR2GRAY)
        image_reshape = np.reshape(image_gray, (1, 784))
        # predicting image label using neural network
        prediction = self.sess.run(self.Y, feed_dict={self.X: image_reshape, self.keep_prob: 1.0})
        index = np.argmax(prediction, 1)[0]
        # publishing topic
        sign = ['RIGHT', 'LEFT', 'TOP', 'BOTTOM', 'UNDER20', 'UNDER50', 'STOP', 'WARNING']
        if prediction[0][index] > self.softmax_threshold:
            print prediction[0][index]
            message = sign[index]
            self._pub.publish(message)
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('signal_sign_dection')
    traffic_sign_dection = TrafficSignDetection()
    traffic_sign_dection.main()
