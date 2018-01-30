# Car barrier detection


This node detects car barrier in 3 steps
 1. Filtering by color
 2. Filtering by size and shape
 3. Testing linearity and distance equality of 3 dots
 4. Calculating angle and distance from your 

#### subscribed topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
 * /image_calibrated(sensor_msgs/Image)
#### published topic
 * /car_barrier(std_msgs/String)
#### parameters
```python
        self.selecting_sub_image = "raw"  # you can choose image type "compressed", "raw"
        self.track_bar = 'on'
        self.image_showing = 'on'
```

## Filtering by color
The first thing to do is filtering original image using mask and this process will be done in HSV coordinate not in RGB one because HSV coordinate is more easy to select typical color range. Next, I conver filtered image to binary image to reduce processed data.


<p align="center">
 <img src="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/1.gif?raw=true" alt="Overview" width="50%" height="50%">
</p>

## Filtering by size and shape
I used 'blob' function to recognize red rectangular. There are many parameters in blob functino but I used these 4 parameter in my project.
 * Threshold
 * Area
 * Circularity
 * Convexity

In the below image you can see only rectangular shape is marked with blue circle
<p align="center">
 <img src="https://github.com/Kihoon0716/self_driving-loading/blob/master/img/1.png?raw=true" alt="Overview" width="50%" height="50%">
</p>

## Testing linearity and distance equality of 3 dots
Last process is testing linearity and distance equality of 3 dots

 * test_linearity(point1, point2, point3) # testing whether 3 dots are located linearly
 * test_distance_equality(point1, point2, point3) # testing whether distance between point1 and point2 is same with distance between point2 and point3

## Calculating angle and distance from your robot
 1.  finding two outer points and drawing blue line
 2.  calculating distance between robot and stop bar using the length of blue line
 3.  calculating angle of line using atan2 function

<a href="https://www.youtube.com/embed/7qObtG6nZL0" target="_blank"><img src="http://img.youtube.com/vi/7qObtG6nZL0/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>


## Result
<a href="https://www.youtube.com/embed/pVKA_5ddetc" target="_blank"><img src="http://img.youtube.com/vi/pVKA_5ddetc/0.jpg" 
alt="circuit" width="480" height="260" border="10" /></a>
