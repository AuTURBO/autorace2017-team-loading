# signal sign detection

The purpose of node is to find signal sign how to use machine learning(tensor flower).

This node detects signal sign detection in 4 steps
 1. Filtering by color(Setting the ROI(region of interest))
 	* bilateralFilter, Canny, dilate, findContours
 2. Filtering by shape
 	* Compare area / Find 4 edge
 3. Find signal sign shape
    * Use machine learning
 4. Publish message


<a href="https://www.youtube.com/embed/53aA34iqnFo" target="_blank"><img src="http://img.youtube.com/vi/53aA34iqnFo/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>

#### subscribed topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
#### published topic
 * /traffic_sign(std_msgs/String)


## Filtering by color(Setting the ROI)
```python
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
```

## Filtering by shape
```python
	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	# do this process if contour have 4 edge
	if len(approx) == 4:
	screenCnt = approx
	area_now = cv2.contourArea(c)
	check_rectangular = distinguish_rectangular(screenCnt)
	# do this process if all angles of rectangular between 70 and 110
	if check_rectangular == 'yes' and area_pre - area_now < 10000:
		cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3) # drawing rectangular box
```

## Image distinction using tensorflow
```python
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
```

## publish message 
```python
# publishing topic
	sign = ['RIGHT', 'LEFT', 'TOP', 'BOTTOM', 'UNDER20', 'UNDER50', 'STOP', 'WARNING']
	if prediction[0][index] > self.softmax_threshold:
		print prediction[0][index]
		message = sign[index]
		self._pub.publish(message)
```
## Try more!!
Our algorithm is very accurate if the signal sign is rectangular shape but can not be used if other shape. You can download open source from [here!](http://wiki.ros.org/find_object_2d) and try this. I made "core.py" can subscribe "objects" topic and can respond to this message. Your robot will slowdown the speed.
