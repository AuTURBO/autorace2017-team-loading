# Parking


#### subscribed topic
 * /image_birdeye_compressed(sensor_msgs/CompressedImage)
 * /image_birdeye(sensor_msgs/Image)
 * /scan(sensor_msgs/LaserScan)
 * /odom(nav_msgs/Odometry)
#### published topic
 * /parking(std_msgs/String)
 * /cmd_vel(geometry_msgs/Twist)

#### parameters

```python
    self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
    self.image_show = 'off'  # monitering image
```

## Flow chart
<p align="center">
 <img src="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/11.png?raw=true" alt="Overview" width="100%" height="100%">
</p>

## Result
<a href="https://www.youtube.com/embed/m0W2h50ZzP8" target="_blank"><img src="http://img.youtube.com/vi/m0W2h50ZzP8/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>
