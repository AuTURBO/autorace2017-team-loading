#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Lane_tracer():
    def __init__(self):

        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.image_show = 'off'  # monitering image

        # subscribers
        if self.selecting_sub_image == "compressed":
            self._sub_1 = rospy.Subscriber('/image_birdeye_compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub_1 = rospy.Subscriber('/image_birdeye', Image, self.callback, queue_size=1)
        self._sub_2 = rospy.Subscriber('/cmd/vel', Twist, self.callback2, queue_size=1)

        self._cv_bridge = CvBridge()

        self.speed = None
        self.angle = None


    def callback(self, image_msg):

        if self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        else:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "mono8")
        cv_image = cv2.resize(cv_image,(28,28))
        np_image = np.reshape(cv_image, 784)

        f = open('TrainingData.txt', 'a')
        for i in range(0,784):
            data = "%f" % np_image[i]
            f.write(data)
            f.write(",")
        data = "%f" % self.angle * 0.06/self.speed
        f.write(data)
        f.write("\n")
        f.close()

    def callback2(self, vel):
        self.speed = vel.linear.x
        self.angle = vel.angular.z

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_follower')
    node = Lane_tracer()
    node.main()
