# traffic_light

The purpose of node is to find traffic light. And this node clssify traffic light color(Red, Green, Orange) after find traffic light.

This node detects traffic light detection in 3 steps

 1. Decoding compressed image
	* imdecode
 2. Detect traffic light(We have 2 methods to detect traffic light)
	* Traffic light1 (First method. How to use CascadeClassifiler)
	* Traffic light2 (Second method. How to use Blob detect)
 3. Publish message

<a href="https://www.youtube.com/embed/Vph3WS3n2xY" target="_blank"><img src="http://img.youtube.com/vi/Vph3WS3n2xY/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>


#### subscribed topic
 * /usb_cam/image_raw/compressed(sensor_msgs/CompressedImage)
#### published topic
 * /traffic_light(self_driving_turtlebot3/Traffi_light)


## Decoding compressed image
Basically when you launch usb_cam node you can see raw image and compressed image is transmitted. However if you want to do image processing in your lab top not in the robot, I recommend you using compressed image because of the time delay.
```python
    import numpy as np  
    from sensor_msgs.msg import CompressedImage
    np_arr = np.fromstring(image_msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
```
In the first line receive the compressed image data as a numpy array and in the next line decode into opencv image format.
Before this process you should import numpy and CompressedImage libraries.

## Traffic light1 (First method. How to use CascadeClassifiler)

### CascadeClassifier(find traffic_light)
```python
	traffic_light_cascade = cv2.CascadeClassifier('cascade.xml')

	gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	traffic_light = traffic_light_cascade.detectMultiScale(gray, 2, 5)
```
Method of CascadeClassifier function need sample data before use the function. You have to train and make cascade.xml file.

### Detect light color(ex)red color)
```python
	hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
	R_mask = cv2.inRange(hsv, R_lower_hsv, R_higher_hsv)
	RedFilter = cv2.bitwise_and(RedFilter, RedFilter, mask=R_mask)
	cv_image_gray = cv2.cvtColor(RedFilter, cv2.COLOR_RGB2GRAY)
	ret,cv_image_binary = cv2.threshold(cv_image_gray,1,255,cv2.THRESH_BINARY_INV)
	cv_image_binary_ = cv2.resize(cv_image_binary,(50,50))
```
If you find traffic light, next you find color. Afte find  detct light color, publish message to next node.


## Traffic light2 (Second method.How to use Blob detect)

### Init variable
- Traffic light color value (red, green, orange)
- Traffic light box and color border distance

### Track bar or init var

### Decode and Convert bgr to hsv
```python
	np_arr = np.fromstring(image_msg.data, np.uint8)
	frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_hsv = np.array([ilowH, ilowS, ilowV])
	higher_hsv = np.array([ihighH, ihighS, ihighV])
	mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
	ColorFilter = frame

	ColorFilter = cv2.bitwise_and(ColorFilter, ColorFilter, mask=mask)
```
If you use compressed msg, you have to do decode. 
And To put up accuracy convert bgr to hsv. Already, Here this part set color what you want to find.

### Blob Detector
```python
	det=cv2.SimpleBlobDetector_create(params)
	keypts=det.detect(cv_image_binary)
```
Detect traffic light.


```python
	point_col = int(keypts[i].pt[0])
	point_low = int(keypts[i].pt[1])
	col_start = point_col - self.black_color_box_size['red']['w']
	col_end = point_col + self.black_color_box_size['red']['w']
	image_crop = frame[low_start:low_end, col_start:col_end]
	image_crop_gray = cv2.cvtColor(image_crop, cv2.COLOR_RGB2GRAY)
	ret, image_crop_binary = cv2.threshold(image_crop_gray, 50, 255, cv2.THRESH_BINARY_INV)
```
Detect traffic light color part.


```python
	black_count = 0
	for i in range(image_crop_binary.shape[1]):
	for j in range(image_crop_binary.shape[0]):
	if image_crop_binary[j][i] == 255:
	black_count += 1
	if black_count > image_crop_binary.shape[0] * image_crop_binary.shape[1] * 0.7:
```
This part is that to reduce mistakes that it find traffic light and color.

