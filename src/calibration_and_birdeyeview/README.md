# Calibration and Bird's eye view

The purpose of this node is to calibrate original image received from usb_cam node and then convert calibrated image to bird's eye view. Calibrated image will be used to recognize traffic-signal or traffic-sign and bird's eye view will be used to to lane following. Each image is pobulished as Image and CompressedImage message so there are totally 4 pobulishers.

#### subscribed topic
 * /usb_cam/image_raw/compressed(sensor_msgs/CompressedImage)
 * /usb_cam/image_raw(sensor_msgs/Image)
#### published topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
 * /image_birdeye_compressed(sensor_msgs/CompressedImage)
 * /image_calibrated(sensor_msgs/Image)
 * /image_birdeye(sensor_msgs/Image)

#### parameters
```python
    self.trackbar = "on" # you can choose showing trackbar or not by "on", "off"
    self.showing_images = "on" # you can choose showing images or not by "on", "off"
    self.selecting_sub_image = "compressed" # you can choose subscriber image type "compressed", "raw"
    self.selecting_pub_image = "raw" # you can choose publisher image type "compressed", "raw"
```

## Decoding compressed image or CvBridge

```python
    if self.selecting_sub_image == "compressed":
        #converting compressed image to opencv image
        np_arr = np.fromstring(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    elif self.selecting_sub_image == "raw":
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
```
Basically when you launch usb_cam node you can see raw image and compressed image is transmitted. However if you want to do image processing in your lab top not in the robot, I recommend you using compressed image because of the time delay.



## Calibration
Before any other image processing the first thing you should is calibration.

```python
    #calibration variables
    fx = 412.939491
    fy = 411.722476
    cx = 309.638220
    cy = 237.289666
 
    k1 = -0.431252
    k2 = 0.138689
    p1 = -0.009537
    p2 = 0.001562
    
    # camera calibration process
    camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
    distortion_matrix = np.array([k1,k2,p1,p2,0])
    cv_image = cv2.undistort(cv_image, camera_matrix, distortion_matrix)
```
To use undistort function of opencv, you need two matrices and to make these matrices you need totally 8 variables.
You might have seen some people moving around black and white chessboard around the camera and you can see how to that in this opencv [tutorial](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)

## Bird's eye view


### homography
Wikipedia says; [any two images of the same planar surface in space are related by a homography.](https://en.wikipedia.org/wiki/Homography_(computer_vision))
That means if you know only 4 variables, you can predict how looks like the image when view point changes.
```python
    h, status = cv2.findHomography(pts_src, pts_dst)
    cv_Homography = cv2.warpPerspective(cv_origin, h, (cv_image.shape[1], cv_image.shape[0]))
```
In the first line findhomography function create homography matrix using pts_src and pts_dst. pts_src contains 4 point of the original image and pts_dst contains corresponding 4 point of changed image

### track bar
I made gui track bar to easily move 4 points of original image and threshold value of binary image.
So you can easily change parameters for your robot!

<p align="center">
 <img src ="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/9.png?raw=true" width="50%" height="50%"/>
</p>

## Result
<p align="center">
 <img src ="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/4.png?raw=true" width="30%" height="30%"/>
<img src ="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/2.png?raw=true" width="30%" height="30%"/>
<img src ="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/3.png?raw=true" width="30%" height="30%"/>
</p>

 * Original image
 * Calibrated image
 * Bird's eye view

```python
    # publishing calbrated and Bird's eye view as compressed image
    msg_calibration = CompressedImage()
    msg_calibration.header.stamp = rospy.Time.now()
    msg_calibration.format = "jpeg"
    msg_calibration.data = np.array(cv2.imencode('.jpg', cv_origin)[1]).tostring()
    self._pub1.publish(msg_calibration)

    msg_homography = CompressedImage()
    msg_homography.header.stamp = rospy.Time.now()
    msg_homography.format = "jpeg"
    msg_homography.data = np.array(cv2.imencode('.jpg', cv_Homography)[1]).tostring()
    self._pub2.publish(msg_homography)

    # publishing calbrated and Bird's eye view as raw image
    self._pub3.publish(self.bridge.cv2_to_imgmsg(cv_origin, "bgr8"))
    self._pub4.publish(self.bridge.cv2_to_imgmsg(cv_Homography, "bgr8"))
```

