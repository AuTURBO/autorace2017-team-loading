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
from std_msgs.msg import String
## developping...
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

def theta_dot2dot(start, end):
    theta = math.atan2(end[1]-start[1], end[0]-start[0])
    return theta

def euler_from_quaternion(rot):
    quaternion = (rot)
    theta = tf.transformations.euler_from_quaternion(quaternion)[2] - np.pi / 2
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

        self._sub = rospy.Subscriber('/map', OccupancyGrid, self.callback, queue_size=1)
        self._sub = rospy.Subscriber('/odom', Odometry, self.callback2, queue_size=1)
        self._sub = rospy.Subscriber('/scan', LaserScan, self.callback3, queue_size=1)

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._pub2 = rospy.Publisher('/maze', String, queue_size=1)

        self.state = 'stop' # path_finding, stop, going, direction_setting

        # variables used in maze solve

        self.img = np.zeros((384, 384, 3), np.uint8)
        self.low_position = 0
        self.col_position = 0
        self.destination_low = 0
        self.destination_col = 0
        self.theta = 0
        self.shortest_path = [[0,0]]
        self.path = [0,0]

        # variables used in move to enterance and exit
        self.position_now = None
        self.theta_now = None
        self.scan = None
        self.start_point = None
        self.end_point = None
        self.theta_exit = None


    def define_destination(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.destination_low = y
            self.destination_col = x
            self.state = 'setting_start_and_goal'

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

        if len(self.path) > 1:
            self.img = cv2.line(self.img, (self.path[1], self.path[0]), (self.path[1], self.path[0]), (0, 170, 255), 2)

            for i in range(len(self.shortest_path)):
                self.img = cv2.line(self.img, (self.shortest_path[i][1], self.shortest_path[i][0]), (self.shortest_path[i][1], self.shortest_path[i][0]), (255, 0, 255), 2)
                if i != 0:
                    self.img = cv2.line(self.img, (self.shortest_path[i][1], self.shortest_path[i][0]), (self.shortest_path[i - 1][1], self.shortest_path[i - 1][0]), (0, 255, 255), 1)
                else:
                    self.img = cv2.line(self.img, (self.col_position, self.low_position), (self.shortest_path[i - 1][1], self.shortest_path[i - 1][0]), (0, 255, 255), 1)

    def callback2(self, odometry):
        ## converting odometry to x position, y position and theta
        self.low_position = 192 + int((odometry.pose.pose.position.x) * 20) + 8
        self.col_position = 192 + int((odometry.pose.pose.position.y) * 20) + 9
        self.theta = euler_from_quaternion([odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w])
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        self.theta_now = self.theta+np.pi/2
        if self.theta < 0:
            self.theta = self.theta + np.pi*2
        direction_desired = math.atan2(self.low_position - self.path[0], self.path[1] - self.col_position)
        if direction_desired < 0:
            direction_desired = direction_desired + np.pi*2


        if self.state == "setting_start_and_goal":
            min_distance = 100
            for i in range(0,90):
                if self.scan.ranges[i] < min_distance:
                    min_distance = self.scan.ranges[i]
                    idx_1 = i
            min_distance = 100
            for i in range(270,360):
                if self.scan.ranges[i] < min_distance:
                    min_distance = self.scan.ranges[i]
                    idx_2 = i
            point1 = [self.position_now[0] + self.scan.ranges[idx_1] * math.cos(idx_1 * np.pi/180), self.position_now[1] + self.scan.ranges[idx_1] * math.sin(idx_1 * np.pi/180)]
            point2 = [self.position_now[0] + self.scan.ranges[idx_2] * math.cos(idx_2 * np.pi/180), self.position_now[1] + self.scan.ranges[idx_2] * math.sin(idx_2 * np.pi/180)]
            between_point1_point2 = [point1[0] + point2[0], point1[1] + point2[1]]
            # defining start point
            angle = theta_dot2dot(self.position_now, between_point1_point2)
            self.start_point = [between_point1_point2[0] + math.cos(angle) * 0.1, between_point1_point2[1] + math.sin(angle) * 0.1]
            # defining end point
            angle = theta_dot2dot(point2, point1)
            self.theta_exit = angle
            distance_axis_x = [1.6*math.cos(angle), 1.6*math.sin(angle)]
            distance_axis_y = [1.6*math.cos(angle - np.pi/2), 1.6*math.sin(angle - np.pi/2)]
            self.end_point = [self.start_point[0] + distance_axis_x[0] + distance_axis_y[0], self.start_point[1] + distance_axis_x[1] + distance_axis_y[1]]

            self.destination_col = 200 + int(self.end_point[0]*20)
            self.destination_low = 201 + int(self.end_point[1]*20)

            self.state = "move_to_start_point"

        if self.state == "move_to_start_point":
            self.move_to_some_point(self.position_now, self.theta_now, self.start_point)
            distance_remain = distance_dot2dot(self.position_now[0], self.position_now[1], self.start_point[0], self.start_point[1])
            if distance_remain < 0.02:
                self.state = "path_finding" # now maze solve start!!


        if self.state == 'direction_setting':
            # calculate degree and direction

            if direction_desired > self.theta:
                if direction_desired - self.theta < np.pi:
                    turn_direction = 'left'
                else:
                    turn_direction = 'right'
            else:
                if self.theta - direction_desired < np.pi:
                    turn_direction = 'right'
                else:
                    turn_direction = 'left'
            # publish topic
            difference = abs(direction_desired - self.theta)
            if difference > np.pi:
                difference = np.pi*2 - difference
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

            if turn_direction =='left':
                angular_z = turn_speed
            else:
                angular_z = - turn_speed
            self.publishing_vel(0, 0, angular_z, 0, 0, 0)

        if self.state == 'going':
            a = math.tan(self.theta + np.pi/2)

            b = -1
            c = -a*self.low_position + self.col_position
            distance_expected = distance_dot2line(a, b, c, self.path[0], self.path[1])
            distance_now = distance_dot2dot(self.low_position, self.col_position, self.path[0], self.path[1])
            distance_from_destination = distance_dot2dot(self.low_position, self.col_position, self.destination_low, self.destination_col)
            self.publishing_vel(0, 0, 0, 0.06, 0, 0)

            # print 'expected : ', distance_expected, 'now : ', distance_now
            if distance_expected > 1:
                self.state = 'direction_setting'
            if distance_from_destination == 0:
                self.state = 'stop'
            elif distance_now == 0:
                self.state = 'path_finding'
                self.publishing_vel(0, 0, 0, 0, 0, 0)

        if self.state == 'stop':
            self.publishing_vel(0, 0, 0, 0, 0, 0)
            self.state = 'align to lane'

        elif self.state == 'align to lane':
            if abs(self.theta_now - self.theta_exit) < np.pi/100:
                self.state = 'maze_end'
                message = "maze_end"
                self._pub2.publish(message)
            else:
                self.setting_angle(self.theta_now, self.theta_exit)


    def callback3(self, scan):
        self.scan = scan
        cv2.namedWindow('SLAM')
        cv2.setMouseCallback('SLAM', self.define_destination)
        img_copy = np.zeros((384, 384, 3), np.uint8)
        np.copyto(img_copy, self.img)
        for i in range(360):
            low_scan = int(scan.ranges[i] * math.sin(i*np.pi/180 + self.theta) * 20)
            col_scan = int(scan.ranges[i] * math.cos(i*np.pi/180 + self.theta) * 20)
            img_copy[self.low_position - low_scan][self.col_position + col_scan][0] = 0
            img_copy[self.low_position - low_scan][self.col_position + col_scan][0] = 0
            img_copy[self.low_position - low_scan][self.col_position + col_scan][2] = 255
        img_copy = cv2.line(img_copy, (self.col_position, self.low_position), (self.col_position, self.low_position), (0, 0, 255), 2)
        img_large = cv2.resize(img_copy,(1000,1000))
        cv2.imshow("SLAM", img_copy), cv2.waitKey(1)
        cv2.imshow("SLAM_large", img_large), cv2.waitKey(1)

    def move_to_some_point(self, position_now, theta_now, position_desired):
        theta_desired = theta_dot2dot(position_now, position_desired)
        diff = abs(theta_desired - theta_now)
        if diff > 2*np.pi:
            diff -= 2*np.pi
        if diff > np.pi/100:
            print 'diff', abs(theta_desired - theta_now)
            self.setting_angle(theta_now, theta_desired)
        else:
            self.going_straight()



    def setting_angle(self, theta_now, theta_desired):
        if theta_desired < 0:
            theta_desired += np.pi*2
        print 'setting angle'
        print theta_now
        print theta_desired
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
        if turn_direction == 'left':
            ang_z = turn_speed
        else:
            ang_z = - turn_speed
        self.publishing_vel(0, 0, 0, 0, 0, ang_z)

    def going_straight(self):
        print 'going straight'
        print self.position_now
        print self.theta_now
        print self.position_parking
        self.publishing_vel(0.06, 0, 0, 0, 0, 0)

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
    rospy.init_node('maze_pathfinder')
    mazesolver = Maze_pathfinder()
    mazesolver.main()
