# Autonomous Batmobile
**MAE ECE 148 Team 4**

![image of robocar](https://media.discordapp.net/attachments/1402478035529240616/1413310283971035257/IMG_0712.jpg?ex=68bd7185&is=68bc2005&hm=27813eb02900b6d7515e61e5f6878601fdaff0f8027d589380c8b37351e5280f&=&format=webp&width=1280&height=960)

## Team Members
Karan Humpal *(EC80)* \
Burhanuddin Mamujee *(MC27)* \
Chenhua (Avery) Wu *(EXT. ROBOTICS)* \
Elizabeth Dam *(EC27)*

![image of team](https://media.discordapp.net/attachments/1191295977496256575/1413926938463965224/IMG_5568.jpg?ex=68bdb593&is=68bc6413&hm=16344847ea56bdcef0c8cd25635e32a0340e2d473765894f4c77a0bc919e151c&=&format=webp&width=1203&height=960)
*left to right: Chenhua(Avery) Wu, Elizabeth Dam, Karan Humpal, Burhanuddin Mamujee*

## Overview
Our project has three main features that an autonomous batmobile should at least have. A [stop sign detector](#stop-sign-detector) that stops the car for 3 seconds then continues to move again. A [color detector](#color-detector), that speeds up when it detects green (the Joker), slows down when it detects blue (civilians), and stops on red. The last feature is an [obstacle avoider](#obstacle-avoider). The batmobile detects an obstacle and avoids it.

## What we promised
### Must haves:
* Stop sign detector (OAK-D-lite)
* Color detector (OAK-D-lite)
* Obstacle Avoider (LiDAR)

### Nice to haves:
* Stop sign detector/color detector and lane detection running at the same time
* Having all three features work at the same time

## ROS2 Layout
There were three main components per feature:
* **constant_speed.py**
  * It is used by all three features and publishes a constant speed of 0.4 to the constant_cmd topic.

* ***feature*_detector.py** (ie. stopsign_detector.py)
  * Depending on the feature, it determines the VESC values and publishes them to their respective topics. (ie. stop_cmd for stop sign detector)

* **cmd_arbiter_*feature*.py**
  * Subscribes to the constant_cmd and *feature*_cmd topics
  * Publishes to cmd_vel topic
  * Determines whether to publish the values received from the constant_cmd or the *feature*_cmd to cmd_vel

Features used in **ucsd_robocar_hub2**: 

* **vesc_twist_node.py** to use the values published in cmd_vel
* **sub_lidar_node.py** to receive values from the /scan topic
* **ucsd_robocar_nav2_pkg** to turn on components and launch multiple nodes at once

## Features
### Stop Sign Detector
The stop sign detector stops for 3 seconds when the OAK-D-lite sees a stop sign. We used a [model from Roboflow Universe](https://universe.roboflow.com/sign-detection-h24ey/stop-sign-h0vwm) and implemented it to our ROS2 package. The stopsign_detection_node.py publishes VESC values to stop_cmd that stops the throttle and steering. constant_speed.py publishes a constant speed of 0.4 to constant_cmd. cmd_arbiter.py subscribes to the topics mentioned. It publishes the values from constant_cmd, unless a value from stop_cmd is received, to cmd_vel. vesc_twist_node.py handles the movement of the VESC and is subscribed to cmd_vel.

Nodes:
 * final_project_batmobile
   * stopsign_detection_node.py
   * constant_speed.py
   * cmd_arbiter.py
 * ucsd_robocar_hub2
   * vesc_twist_node.py

### Color Detector
The color detector speeds up when green is detected, slows down when blue is detected, and stops when red is detected. We used OpenCV to filter out and detect the colors. color_detection_final.py detects if a color is seen and then publishes the corresponding VESC values to color_cmd. Then it has its own arbiter, cmd_arbiter_color.py, that uses similar logic as the stop sign's arbiter. It publishes the constant_cmd values to cmd_vel unless a value from color_cmd is received.

Nodes:
 * final_project_batmobile
   * color_detection_final.py
   * constant_speed.py
   * cmd_arbiter_color.py
 * ucsd_robocar_hub2
   * vesc_twist_node.py

### Obstacle Avoider

## Robot Design
### Schematic
![schematic](https://media.discordapp.net/attachments/1191295977496256575/1413932896401625238/image.png?ex=68bdbb20&is=68bc69a0&hm=7a4f4c297bf96f83e92fa3b1374bc1c8f04d7dbb00ad340ad6b7fd2564c11670&=&format=webp&quality=lossless)

### Hardware
Camera Mount
Jetson Mount
Lidar/GPS mount
Top Plate

## If we had another week
If we had another week,

## Past Batmobile Accomplishments
* [Donkey Car Autonomous Laps](https://youtu.be/wXakF6iPv3Q?si=_Q1qe9eq6RRz-gBj)
* [Donkey Car GPS Laps](https://youtu.be/d-aGpujyBqg?si=FyLxy5e6jBKazhFP)
* [Lane Detection Laps using ROS2](https://youtu.be/SRFx1kLqEJU?feature=shared)

## Resources
* [Git repository for ucsd_robocar_hub2](https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2)
* [Model for stopsign detector](https://universe.roboflow.com/sign-detection-h24ey/stop-sign-h0vwm)
* [Jetson Nano case/mount](https://cults3d.com/en/3d-model/tool/jetson-nano-case-abs-edition)

## Acknowledgements


## Contact Information



