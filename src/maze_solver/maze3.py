#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import tf
import numpy as np
from std_msgs.msg import String
import cv2
## developping...


def distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def distance_dot2dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2-y1))
    return distance


def theta_dot2dot(start, end):
    theta = math.atan2(end[1]-start[1], end[0]-start[0])
    return theta

def euler_from_quaternion(rot):
    quaternion = (rot)
    theta = tf.transformations.euler_from_quaternion(quaternion)[2]
    return theta

def sign(num):
    if num < 0:
        return -1
    else:
        return 1



class Orientation(object):
    def __init__(self, trans, rot):
        self.x = trans[0]
        self.y = trans[1]
        self.theta = euler_from_quaternion(rot)


class Maze_pathfinder():
    def __init__(self):

        self._sub = rospy.Subscriber('/odom', Odometry, self.callback2, queue_size=1)
        self._sub = rospy.Subscriber('/scan', LaserScan, self.callback3, queue_size=1)
        self._sub_2 = rospy.Subscriber('/command_maze', String, self.receiver_from_core, queue_size=1)

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._pub2 = rospy.Publisher('/maze', String, queue_size=1)

        #self.state = 'setting_point' # path_finding, stop, going, direction_setting
        self.state = 'waiting'  # path_finding, stop, going, direction_setting
        # variables used in maze solve

        self.sell_size = 0.1
        self.car_size = 0.1


        # variables used in move to enterance and exit
        self.position_now = None
        self.theta_now = None

        self.standard_theta = None

        self.scan = None
        self.start_point = None
        self.exit_point = None
        self.exit_point2 = None

        self.current_direction = None
        self.theta_from_direction = {'right':0, 'top':np.pi/2, 'left':np.pi, 'bottom':np.pi*3/2}
        self.angle_from_direction = {'right':0, 'top':90, 'left':180, 'bottom':270}

        self.moving_point = None
        self.key_point = [[0,0.2],[-1.1,0],[0,1.3],[-0.5, 0],[0,0.2]]
        self.move_point = None
        self.move_coor = None
        self.theta_stan = None

        self.showing_image = "off"


    def callback2(self, odometry):
        #print self.state

        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        quaternion = (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
        self.theta_now = euler_from_quaternion(quaternion)

        #self.theta_now = self.theta_now - self.theta_stan
        if self.theta_now < 0:
            self.theta_now += 2*np.pi


        if self.state == "setting_point":
            if self.theta_stan == None:
                self.theta_stan = self.theta_now
            if len(self.key_point) != 0:
                self.move_point = self.key_point.pop()
                self.move_coor = [self.position_now[0] + math.cos(self.theta_stan - np.pi/2) * self.move_point[0] - math.sin(self.theta_stan - np.pi/2) * self.move_point[1], self.position_now[1] + math.sin(self.theta_stan - np.pi/2) * self.move_point[0] + math.cos(self.theta_stan - np.pi/2) * self.move_point[1]]
                self.state = "moving"
            else:
                self.state = "go_to_wall"



        #print "pose now", self.position_now, "theta now", self.theta_now
        #print "move_coor", self.move_coor
        #print "theta desired", theta_dot2dot(self.position_now, self.move_coor)
#        print "pose_now", self.position_now, "pose_desired", self.move_coor
        if self.state == "moving":
            self.move_to_some_point(self.position_now, self.theta_now, self.move_coor)






    def callback3(self, scan):
        if self.showing_image == "on":
            img = np.zeros((2000, 2000, 3), np.uint8)
            for i in range(360):
                x = 1000 + int(math.cos((i-90)*np.pi/180)*scan.ranges[i] * 200)
                y = 1000 - int(math.sin((i-90)*np.pi/180)*scan.ranges[i] * 200)
                img[y,x,0] = 255
                img[y, x, 1] = 255
                img[y, x, 2] = 255
            img = cv2.circle(img, (1000, 1000), 3, (0, 0, 255), thickness=3, lineType=8, shift=0)
            cv2.imshow('laser', img), cv2.waitKey(1)

        if self.state == "go_to_wall":
            if scan.ranges[0] > 0.17 or scan.ranges[0] == 0:
                self.publishing_vel(0,0,0,0.06,0,0)
            else:
                self.stop()
                self.state = "setting_point"
                self.key_point = [[-2,0]]



    def move_to_some_point(self, position_now, theta_now, position_desired):
        theta_desired = theta_dot2dot(position_now, position_desired)
        if theta_desired < 0:
            theta_desired += np.pi *2

        #print "theta_now", theta_now, "theta_desired", theta_desired
        diff = abs(theta_desired - theta_now)
        distance = distance_dot2dot(position_now[0], position_now[1], position_desired[0], position_desired[1])
        if diff > 2*np.pi:
            diff -= 2*np.pi
        if diff > np.pi/100:
            self.setting_angle(theta_now, theta_desired)
        else:
            self.going_straight()
        if distance < 0.01:
            if self.state == "moving":
                self.state = 'setting_point'
            self.stop()



    def setting_angle(self, theta_now, theta_desired):
        if theta_desired < 0:
            theta_desired += np.pi*2
        if theta_desired > theta_now:
            if theta_desired - theta_now < np.pi:
                turn_direction = 'left'
            else:
                turn_direction = 'right'
        else:
            if theta_now - theta_desired < np.pi:
                turn_direction = 'right'
            else:
                turn_direction = 'left'
                # publish topic
        difference = abs(theta_desired - theta_now)
        if difference > np.pi:
            difference = np.pi * 2 - difference
        if difference > 0.1:
            turn_speed = 0.6
        elif difference > 0.01:
            turn_speed = 0.1
        else:
            turn_speed = 0
        if turn_direction == 'left':
            ang_z = turn_speed
        else:
            ang_z = - turn_speed
        self.publishing_vel(0, 0, ang_z, 0, 0, 0)

    def going_straight(self):
        self.publishing_vel(0, 0, 0, 0.12, 0, 0)

    def stop(self):
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

    def receiver_from_core(self, command):
        if command.data == "maze_start":
            self.state = "setting_point"
        if command.data == "maze_end":
            self.stop()
            self.state = "waiting"


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('maze3')
    mazesolver = Maze_pathfinder()
    mazesolver.main()
