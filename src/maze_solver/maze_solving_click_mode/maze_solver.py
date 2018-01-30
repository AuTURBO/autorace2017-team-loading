#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from compressed_image_transport import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf
import math
import cv2
import numpy as np
from maze_solving_algorithm import Solver
import threading


def existance(arr, num):
    for i in range(0, len(arr)):
        if arr[i] == num:
            return True
    return False

def configure(arr):
    arr_ = []
    for i in range(0, len(arr)):
        if existance(arr_, arr[i]) == False:
            arr_.append(arr[i])
    return arr_

def distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def distance_dot2dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2-y1))
    return distance

def collision_test(start, goal, map, difference_low, difference_col):
    start = [start[0] - difference_low, start[1] - difference_col]
    goal = [goal[0] - difference_low, goal[1] - difference_col]
    if goal[0] != start[0]:
        a = (goal[1] - start[1]) / (goal[0] - start[0])
        b = -a*start[0] + start[1]
        for i in range(min(start[0], goal[0]), max(start[0], goal[0])):
            if map[i][int(a*i + b)] == True:
                return 'danger'
    else:
        for i in range(min(start[1], goal[1]), max(start[1], goal[1])):
            if map[start[0]][i] == True:
                return 'danger'
    return 'safe'

def euler_from_quaternion(quaternion):
    theta = tf.transformations.euler_from_quaternion(quaternion)[2] - 3.141592 / 2
    if theta < 0:
        theta = theta + 3.141592 * 2
    return theta





class Maze_pathfinder():
    def __init__(self):

        self._sub = rospy.Subscriber('/map', OccupancyGrid, self.callback, queue_size=1)
        self._sub = rospy.Subscriber('/odom', Odometry, self.callback2, queue_size=1)
        self._sub = rospy.Subscriber('/scan', LaserScan, self.callback3, queue_size=1)
        # self._sub = rospy.Subscriber('/tf', TFMessage, self.callback4, queue_size=1)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.img = np.zeros((384, 384, 3), np.uint8)
        self.low_position = 0
        self.col_position = 0
        self.destination_low = 0
        self.destination_col = 0
        self.theta = 0
        self.state = 'stop' # path_finding, stop, going, direction_setting
        self.shortest_path = [[0,0]]
        self.path = [0,0]




    def define_destination(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.destination_low = y
            self.destination_col = x
            self.state = 'path_finding'

    def callback(self, map):
        thread1 = threading.Thread(target=self.path_finding, args=(map,))
        thread1.setDaemon(True)
        thread1.start()

    def path_finding(self, map):
        self.img = np.zeros((384, 384, 3), np.uint8)

        for i in range(0, 384):
            for j in range(0, 384):
                if map.data[384*j + i] == -1:
                    self.img[i][j][0] = 255
                if map.data[384*j + i] == 0:
                    self.img[i][j][1] = 255
                if map.data[384*j + i] == 100:
                    self.img[i][j][0] = 0
                    self.img[i][j][1] = 0
                    self.img[i][j][2] = 0
        # Draw direction
        self.img = cv2.line(self.img, (self.col_position, self.low_position), (self.col_position + int(10*math.cos(self.theta)), self.low_position - int(10*math.sin(self.theta))), (0, 255, 255), 1)

        if self.state == 'path_finding':
            print 'path finding....'
            solver = Solver([self.low_position, self.col_position], [self.destination_low, self.destination_col], map.data)
            solver.solve_distance()
            solver.find_shortest_path()
            self.shortest_path = solver.shortest_path
            self.state = 'direction_setting'
            print 'path finding end!'

            while(1):
                if len(self.shortest_path) > 2:
                    if solver.collision_test([self.low_position, self.col_position], self.shortest_path[len(self.shortest_path)-3]) == 'safe':
                        _ = self.shortest_path.pop()
                        print 'poped out'
                    else:
                        print 'collision'
                        break
                else:
                    break
            self.path = self.shortest_path[len(self.shortest_path)-2]
            if self.path == [0,0]:
                print "something wrong!"

        if self.state != 'stop' and self.state != 'path_finding':
            self.img = cv2.line(self.img, (self.path[1], self.path[0]), (self.path[1], self.path[0]), (0, 170, 255), 2)

            for i in range(len(self.shortest_path)):
                self.img = cv2.line(self.img, (self.shortest_path[i][1], self.shortest_path[i][0]), (self.shortest_path[i][1], self.shortest_path[i][0]), (255, 0, 255), 2)
                if i != 0:
                    self.img = cv2.line(self.img, (self.shortest_path[i][1], self.shortest_path[i][0]), (self.shortest_path[i - 1][1], self.shortest_path[i - 1][0]), (0, 255, 255), 1)
                else:
                    self.img = cv2.line(self.img, (self.col_position, self.low_position), (self.shortest_path[i - 1][1], self.shortest_path[i - 1][0]), (0, 255, 255), 1)

    def callback2(self, odometry):
        #print 'map_to_odom', self.tf_map_to_odom[0], self.tf_map_to_odom[1]
        #print 'odom_to_base', self.tf_odom_to_base[0], self.tf_odom_to_base[1]
        #print 'odom', odometry.pose.pose.position.x, odometry.pose.pose.position.y
        #print self.tf_map_to_odom[0] + self.tf_odom_to_base[0], self.tf_map_to_odom[1] + self.tf_odom_to_base[1]

        #quaternion = (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
        #self.theta = euler_from_quaternion(quaternion)

        direction_desired = math.atan2(self.low_position - self.path[0], self.path[1] - self.col_position)
        if direction_desired < 0:
            direction_desired = direction_desired + 3.141592*2


        if self.state == 'direction_setting':
            # calculate degree and direction

            if direction_desired > self.theta:
                if direction_desired - self.theta < 3.141592:
                    turn_direction = 'left'
                else:
                    turn_direction = 'right'
            else:
                if self.theta - direction_desired < 3.141592:
                    turn_direction = 'right'
                else:
                    turn_direction = 'left'
            # publish topic
            difference = abs(direction_desired - self.theta)
            if difference > 3.141592:
                difference = 3.141592*2 - difference
            if difference > 0.3:
                turn_speed = 0.6
            elif difference > 0.2:
                turn_speed = 0.3
            elif difference > 0.1:
                turn_speed = 0.1
            elif difference > 0.01:
                turn_speed = 0.05
            else:
                turn_speed = 0
                self.state = 'going'
            vel = Twist()
            if turn_direction =='left':
                vel.angular.z = turn_speed
            else:
                vel.angular.z = - turn_speed
            vel.angular.x = 0
            vel.angular.y = 0
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            self._pub.publish(vel)


        if self.state == 'going':
            a = math.tan(self.theta + 3.141592/2)
            b = -1
            c = -a*self.low_position + self.col_position
            distance_expected = distance_dot2line(a, b, c, self.path[0], self.path[1])
            distance_now = distance_dot2dot(self.low_position, self.col_position, self.path[0], self.path[1])
            distance_from_destination = distance_dot2dot(self.low_position, self.col_position, self.destination_low, self.destination_col)
            # print 'expected : ', distance_expected, 'now : ', distance_now
            if distance_expected > 1:
                self.state = 'direction_setting'
            if distance_from_destination == 0:
                self.state = 'stop'
            elif distance_now == 0:
                self.state = 'path_finding'
            vel = Twist()
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            vel.linear.x = 0.06
            vel.linear.y = 0
            vel.linear.z = 0
            self._pub.publish(vel)

        if self.state == 'stop':
            vel = Twist()
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            self._pub.publish(vel)


    def callback3(self, scan):
        cv2.namedWindow('SLAM')
        cv2.setMouseCallback('SLAM', self.define_destination)
        img_copy = np.zeros((384, 384, 3), np.uint8)
        np.copyto(img_copy, self.img)
        for i in range(360):
            low_scan = int(scan.ranges[i] * math.sin(i*3.141592/180 + self.theta) * 20)
            col_scan = int(scan.ranges[i] * math.cos(i*3.141592/180 + self.theta) * 20)
            img_copy[self.low_position - low_scan][self.col_position + col_scan][0] = 0
            img_copy[self.low_position - low_scan][self.col_position + col_scan][1] = 0
            img_copy[self.low_position - low_scan][self.col_position + col_scan][2] = 255
        img_copy = cv2.line(img_copy, (self.col_position, self.low_position), (self.col_position, self.low_position), (0, 0, 255), 2)
        img_large = cv2.resize(img_copy,(1000,1000))
        cv2.imshow("SLAM", img_copy), cv2.waitKey(1)
        cv2.imshow("SLAM_large", img_large), cv2.waitKey(1)

    def tf_listener_map_to_base(self):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.low_position = 192 + int((trans[0]) * 20) + 7
                self.col_position = 192 + int((trans[1]) * 20) + 8
                self.theta = euler_from_quaternion(rot)
                print 'basefootprint', trans, rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()



    def main(self):
        thread_tf_listener_map_to_base = threading.Thread(target = self.tf_listener_map_to_base)
        thread_tf_listener_map_to_base.start()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('maze_pathfinder')
    mazesolver = Maze_pathfinder()
    mazesolver.main()
