
import cv2
import numpy as np



class ObstacleCreator(object):
    def __init__(self, size):
    	self.map = np.zeros((size[0], size[1]), dtype=np.bool) # obstacles

        self.lows = size[0]
        self.cols = size[1]

	def create_obstalces(event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.map[y/100, x/100] = not self.map[y/100, x/100]

	cv2.namedWindow('image')
	cv2.setMouseCallback('image',create_obstalces)
