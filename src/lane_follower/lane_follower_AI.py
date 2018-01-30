#!/usr/bin/env python
import math
from std_msgs.msg import String
import cv2
import numpy as np
import rospy
import tensorflow as tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


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

class Lane_tracer_AI():
    def __init__(self):

        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.image_show = 'off'  # monitering image

        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub_1 = rospy.Subscriber('/image_birdeye_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub_1 = rospy.Subscriber('/image_birdeye', Image, self.callback, queue_size=1)
        self._sub_2 = rospy.Subscriber('/command_lane_follower', String, self.receiver_from_core, queue_size=1)

        # publishers
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        self._cv_bridge = CvBridge()

        self.run = 'yes'

        self.speed = 2 # Increasing this variable makes angular and linear speed fast in same ratio


    def callback(self, image_msg):
        if self.run == 'stop':
            print 'stop!'
            return

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        else:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "mono8")
        cv_image = cv2.resize(cv_image,(28,28))
        np_image = np.reshape(cv_image, 784)
        prediction = self.sess.run(self.Y, feed_dict={self.X: np_image, self.keep_prob: 1.0})
        angular_z = prediction

        if self.image_show == 'on':
            # showing image
            cv2.imshow('tracer', cv_image), cv2.waitKey(1)

        # poblishing cmd_vel topic
        self.publishing_vel(0, 0, angular_z, 0.06 * self.speed, 0, 0)


    def receiver_from_core(self, command):
        self.run = command.data
        if self.run == 'slowdown':
            self.speed = 1
        elif self.run == 'go':
            self.speed = 2

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
    node = Lane_tracer_AI()
    node.main()
