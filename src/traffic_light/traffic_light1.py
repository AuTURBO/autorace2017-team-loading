#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import math
from self_driving_turtlebot3.msg import Traffic_light


traffic_light_cascade = cv2.CascadeClassifier('cascade.xml')

# Red
R_lowH = 0
R_highH = 255
R_lowS = 0
R_highS = 255
R_lowV = 0
R_highV = 255

R_lower_hsv = np.array([R_lowH, R_lowS, R_lowV])
R_higher_hsv = np.array([R_highH, R_highS, R_highV])

# Green
G_lowH = 75
G_highH = 100
G_lowS = 60
G_highS = 255
G_lowV = 180
G_highV = 255

G_lower_hsv = np.array([G_lowH, G_lowS, G_lowV])
G_higher_hsv = np.array([G_highH, G_highS, G_highV])

# Orange
O_lowH = 10
O_highH = 50
O_lowS = 50
O_highS = 220
O_lowV = 140
O_highV = 255

O_lower_hsv = np.array([O_lowH, O_lowS, O_lowV])
O_higher_hsv = np.array([O_highH, O_highS, O_highV])


class Traffic_light_detection():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.callback, queue_size=1)
        #self._sub = rospy.Subscriber('image_calibrated_compressed', CompressedImage, self.callback, queue_size=1)
        #self._pub = rospy.Publisher('/traffic_light', String, queue_size=1)
        self._pub = rospy.Publisher('/traffic_light', Traffic_light, queue_size=1)
        self.count = 0

    def callback(self, image_msg):
        self.count += 1
        if self.count%3 == 0:
            np_arr = np.fromstring(image_msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            traffic_light = traffic_light_cascade.detectMultiScale(gray, 2, 5)
            for (x, y, w, h) in traffic_light:
                cv2.rectangle(img, (x,y), (x+w, y+h), (225, 0, 0), 2)
                img_crop = img[y:y+h, x:x+w]
                hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
                #detect Red light
                R_mask = cv2.inRange(hsv, R_lower_hsv, R_higher_hsv)
                RedFilter = img_crop
                RedFilter = cv2.bitwise_and(RedFilter, RedFilter, mask=R_mask)
                cv_image_gray = cv2.cvtColor(RedFilter, cv2.COLOR_RGB2GRAY)
                ret,cv_image_binary = cv2.threshold(cv_image_gray,1,255,cv2.THRESH_BINARY_INV)
                cv_image_binary_ = cv2.resize(cv_image_binary,(50,50))
                Red_count = 0
                cv2.imshow("RedFilter", cv_image_binary), cv2.waitKey(1)

                for i in range(cv_image_binary_.shape[0]):
                    for j in range(cv_image_binary_.shape[1]):
                        if cv_image_binary_[i][j] == 0:
                            Red_count += 1
                if Red_count/float(cv_image_binary_.shape[0]*cv_image_binary_.shape[1]) > 0.01:
                    #message = "Red"
                    #self._pub.publish(message)
                    message = Traffic_light()
                    message.color = 'red'
                    message.position_x = x
                    message.position_y = y
                    message.width = w
                    message.height = h
                    self._pub.publish(message)

                #detect Green light
                G_mask = cv2.inRange(hsv, G_lower_hsv, G_higher_hsv)
                GreenFilter = img_crop
                GreenFilter = cv2.bitwise_and(GreenFilter, GreenFilter, mask=G_mask)
                cv_image_gray = cv2.cvtColor(GreenFilter, cv2.COLOR_RGB2GRAY)
                ret,cv_image_binary = cv2.threshold(cv_image_gray,1,255,cv2.THRESH_BINARY_INV)
                cv_image_binary_ = cv2.resize(cv_image_binary,(50,50))
                Green_count = 0
                cv2.imshow("GreenFilter", cv_image_binary), cv2.waitKey(1)

                for i in range(cv_image_binary_.shape[0]):
                    for j in range(cv_image_binary_.shape[1]):
                        if cv_image_binary_[i][j] == 0:
                            Green_count += 1
                if Green_count/float(cv_image_binary_.shape[0]*cv_image_binary_.shape[1]) > 0.01:
                    message = "Green"
                    self._pub.publish(message)
            cv2.imshow("signal light", img), cv2.waitKey(1)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light1')
    node = Traffic_light_detection()
    node.main()
