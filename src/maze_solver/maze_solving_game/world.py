
import cv2
import numpy as np

class World(object):
    def __init__(self, size, map, distance, parent, shortest_path):
        self.lows = size[0]
        self.cols = size[1]
        self.map = map
        self.img = np.zeros((self.lows*100, self.cols*100,3), np.uint8)
        self.distance = distance
        self.blink_space = np.zeros((self.lows*100, self.cols*100,3), np.uint8)
        self.parent = parent
        self.shortest_path = shortest_path
        for i in range(0, self.lows*100):
            for j in range(0, self.cols*100):
                if i%100 == 0 or j%100 == 0:
    				self.blink_space[i,j,0] = 255
    				self.blink_space[i,j,1] = 255
    				self.blink_space[i,j,2] = 255

    def world_reset(self):
        self.img = np.copy(self.blink_space)



    def draw_distance(self):
        for i in range(0, self.lows):
            for j in range(0, self.cols):
                if self.map[i, j] == False:
                    cv2.putText(self.img, '%d' %self.distance[i,j,0], (j*100 + 10, i*100 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255), 1)
                    cv2.putText(self.img, '%d' %self.distance[i,j,1], (j*100 + 60, i*100 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255), 1)
                    cv2.putText(self.img, '%d' %self.distance[i,j,2] ,(j*100 + 10, i*100 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 0, 255), 1)
                    cv2.putText(self.img, '%d' %self.distance[i,j,3] ,(j*100 + 60, i*100 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255, 0, 0), 1)
                    cv2.putText(self.img, '%d' %self.parent[i,j,0] ,(j*100 + 10, i*100 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 255, 0), 1)
                    cv2.putText(self.img, '%d' %self.parent[i,j,1] ,(j*100 + 60, i*100 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 255, 0), 1)
                else:
                    self.img[i*100+1:i*100+99, j*100+1:j*100+99, 1] = 255

    def draw_parent(self):
        for i in range(0, self.lows):
            for j in range(0, self.cols):
                if self.map[i][j] == False:
                    cv2.line(self.img, (j*100+50, i*100+50), (self.parent[i][j][1]*100 + 50, self.parent[i][j][0]*100 + 50), (255, 255, 0), 1)

    def draw_shortest_path(self):
        for i in range(len(self.shortest_path)-1):
            cv2.line(self.img, (self.shortest_path[i][1]*100+50, self.shortest_path[i][0]*100+50), (self.shortest_path[i+1][1]*100 + 50, self.shortest_path[i+1][0]*100 + 50), (255, 255, 255), 1)
