# Core


#### subscribed topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
 * /image_calibrated(sensor_msgs/Image)
 * /stop_bar(self_driving_turtlebot3/Stop_bar)
 * /traffic_light(self_driving_turtlebot3/Traffic_light)
 * /parking(std_msgs/String)
 * /scan(sensor_msgs/LaserScan)
 * /maze(std_msgs/String)
 * /signal_sign(std_msgs/String)
 * /objects(std_msgs/Float32MultiArray)

#### published topic
 * /command_lane_follower(std_msgs/String)

#### parameters

```python
    self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
    self.image_show = 'yes'  # monitering image
```

## rqt_graph
<p align="center">
 <img src="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/10.png?raw=true" alt="Overview" width="100%" height="100%">
</p>

