# Lane follower


This node consisted of this 2 steps
 1. Lane detection
 2. PID control
 3. Calculating angle of bar

Our goal is to make program more easy and light. First, we think obout using houghline algorithm of ovencv because many people use that. However, when we saw Bird's eye view we can know that we do not need to use houghline because Bird'eye view set the ROI automatically and there is no noise in the track. Our algorithm is much more fast and accurate houghline one. Let' see this video
<a href="https://youtu.be/lUix89Cpf3g" target="_blank"><img src="http://img.youtube.com/vi/lUix89Cpf3g/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>
#### subscribed topic
 * /image_birdeye_compressed(sensor_msgs/CompressedImage)
 * /image_birdeye(sensor_msgs/Image)
#### published topic
 * /cmd_vel(geometry_msgs/Twist)

#### parameters

```python
    self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
    self.image_show = 'on'  # monitering image
```


## Lane detection
<p align="center">
 <img src="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/5.png?raw=true" alt="Overview" width="50%" height="50%">
</p>

```python
    low_point1 = 599

    while(1):
        try:
            _, col_point1 =  centroid(cv_image, low_point1, 300, cv_image.shape[1])
            break
        except Exception as e:
            if low_point1 < 200:
                self.lane_existance = 'no'
                break
            else:
                low_point1 -= 50

    low_point2 = low_point1 - 50
    while (1):
        try:
            _, col_point2 = centroid(cv_image, low_point2, 300, cv_image.shape[1])
            self.lane_existance = 'yes'
            break
        except Exception as e:
            if low_point2 < 150:
                self.lane_existance = 'no'
                break
            else:
                low_point2 -= 50
```
I choosed using only right side line so sat the ROI as a right side of image. This code detect center of the lane from the bottom and connect 2 detected points and I will call this as "desired path"

## PID control
```python
    def PIDcontrol(self, x0, y0, x1, y1, x2, y2):
        Ip = 0.2
        distance = 350
        if x1 != x2:
            a = (float(y2)-float(y1))/(float(x2)-float(x1))
        else:
            a = (float(y2)-float(y1))/1
        b = -1
        c = -a*x1 + y1
        theta_current = -math.atan(1/a)*180/np.pi
        sign_, err = distance_dot2line(a, b, c, x0, y0)
        theta_desired = (err - distance) * Ip * sign_
        theta_wheel = (theta_desired - theta_current) * 0.005 * self.speed # origin : 0.005 next : 0.007
        return sign_, theta_wheel
```
the error means distance between robot and desired path and our goal is to maintain error consistantly in a some value. PID control is very simple. More error means more angular valocity!
Lastly, we should consider the current direction of robot which is theta_current. 


## Exception
```python
        if self.lane_existance == 'no':
            print "no lane"
            # showing binary and original images
            if self.image_show == 'on':
                cv2.imshow('tracer', cv_image), cv2.waitKey(1)

            angular_z = self.lane_position * 0.2 * self.speed
            linear_x = 0.06 * self.speed
            self.publishing_vel(0, 0, angular_z, linear_x, 0, 0)
            return
```
There is some situation like when our robot turn right and can not see any lane because our ROI if right side. So, if there is no lane detected our robot will turn right until he will find lane.


## Try more!!
In this derectory I uploaded 3 other files. Be carefull! Only "lane_follower_AI.py" is ROS node others will be used to train neural network.
 * lane_follower_AI.py
 * lane_follower_AI_collect_data.py
 * train.py

These are what I made lane_follower version using tensorflow.
Before trying this you should install tensorflow in [here](https://www.tensorflow.org/install/)
1. By using "lane_follower_IA_collect_data.py" you can collect training data. Inputs are cmd_vel and Image topics and it will create text file containing image array and related angular velocity.
2. Secondly, using "train.py", you can train the neural network and this will create trained model data which will be used in next stage.
3. Finally, run the "lane_follower_AI.py" node and before doing this you should chomd +x model.ckpt in derectory saved the files.
