# **Autonomous Batmobile**
**MAE ECE 148 Team 4**

<p align="center">
  <img width="1000" height="1000" src="https://static0.cbrimages.com/wordpress/wp-content/uploads/2022/11/bat-signal-batman-89.jpg?q=50&fit=crop&w=825&dpr=1.5">
</p>

## Team Members
Karan Humpal *(EC80)* \
Burhanuddin Mamujee *(MC27)* \
Chenhua (Avery) Wu *(EXT. ROBOTICS)* \
Elizabeth Dam *(EC27)*

<p align="center">
  <img width="720" height="1800" src="https://media.discordapp.net/attachments/1191295977496256575/1413926938463965224/IMG_5568.jpg?ex=68bdb593&is=68bc6413&hm=16344847ea56bdcef0c8cd25635e32a0340e2d473765894f4c77a0bc919e151c&=&format=webp&width=1203&height=960">
</p>

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

[Stop Sign Detector In Action](https://youtu.be/6DBiqFDt8yI)

Nodes:
 * final_project_batmobile
   * stopsign_detection_node.py
     * Publishers: stop_cmd
   * constant_speed.py
     * Publishers: constant_cmd
   * cmd_arbiter.py
     * Subscribers: stop_cmd, constant_cmd
     * Publishers: cmd_vel
 * ucsd_robocar_hub2
   * vesc_twist_node.py
     * Subscribers: cmd_vel
    
Issues We Faced:
We faced power supply issues with our Jetson from the DC-DC converter and an unreliable Jetson, so we had to get them replaced. After that, the only problems faced were learning how to use ROS2 to implement our ideas, as it was our first time making our own ROS2 packages. We attempted to connect to the VESC directly through our Python executable files, but were unsuccessful. Instead, we opted to turn the VESC on using the provided ucsd_robocar_hub2.

### Color Detector
The color detector speeds up when green is detected, slows down when blue is detected, and stops when red is detected. We used OpenCV to filter out and detect the colors. color_detection_final.py detects if a color is seen and then publishes the corresponding VESC values to color_cmd. Then it has its own arbiter, cmd_arbiter_color.py, that uses similar logic as the stop sign's arbiter. It publishes the constant_cmd values to cmd_vel unless a value from color_cmd is received.

[Color Detector In Action](https://youtu.be/pb56wiNUj_I)

Nodes:
 * final_project_batmobile
   * color_detection_final.py
     * Publishers: color_cmd
   * constant_speed.py
     * Publishers: constant_cmd
   * cmd_arbiter_color.py
     * Subscribers: color_cmd, constant_cmd
     * Publishers: cmd_vel
 * ucsd_robocar_hub2
   * vesc_twist_node.py
     * Subscribers: cmd_vel

Issues We Faced:
After all the troubleshooting with the stop sign detector, the implementation of the color detector was smooth sailing due to their similar nature.

### Obstacle Avoider
The obstacle avoider uses the LiDAR to detect obstacles. lidar_detection_final.py subscribes to to /scan topic, which receives messages from sub_lidar_node.py. We determined the angle that is considered the front of the robot and computes VESC values to avoid the obstacle (depending on if it is on the right/left/center). The VESC values get published to lider_cmd. cmd_arbiter_lidar.py uses the constant_cmd values unless a value from lidar_cmd is given. The end value gets published to cmd_vel for the vest_twist_node.py to handle.

[Obstacle Avoider In Action](https://youtu.be/g9mnh_GOr-M)

Nodes:
 * final_project_batmobile
   * lidar_detection_final.py
     * Subscribers: /scan
     * Publishers: lidar_cmd
   * constant_speed.py
     * Publishers: constant_cmd
   * cmd_arbiter_lidar.py
     * Subscribers: lidar_cmd, constant_cmd
     * Publishers: cmd_vel
 * ucsd_robocar_hub2
   * vesc_twist_node.py
     * Subscribers: cmd_vel
   * sub_lider_node:
     * Publishers: /scan

Issues We Faced:
For the obstacle detector, it was challenging to determine how the car should respond depending on its distance from the obstacle and what actions it should undertake. It took multiple attempts to fine-tune the car to determine the speed at which it would turn, the direction it would turn, and when the car should turn.

## Robot Design
![image of robocar](https://media.discordapp.net/attachments/1402478035529240616/1413310283971035257/IMG_0712.jpg?ex=68bd7185&is=68bc2005&hm=27813eb02900b6d7515e61e5f6878601fdaff0f8027d589380c8b37351e5280f&=&format=webp&width=1280&height=960)

### Schematic
![schematic](https://media.discordapp.net/attachments/1191295977496256575/1413932896401625238/image.png?ex=68bdbb20&is=68bc69a0&hm=7a4f4c297bf96f83e92fa3b1374bc1c8f04d7dbb00ad340ad6b7fd2564c11670&=&format=webp&quality=lossless)

### Hardware
| Part  | CAD Design |
| ------------- | ------------- |
| Camera Mount  | ![cameramount](https://media.discordapp.net/attachments/1402478035529240616/1413634269309304993/image.png?ex=68bd4dc1&is=68bbfc41&hm=c89259f8ec31abf47c62be5a7b2d1d6547cefd3adaaab574db6d5af610cc97f4&=&format=webp&quality=lossless)  |
| LiDAR/GPS Mount  | ![lidar/gps](https://media.discordapp.net/attachments/1402478035529240616/1413312468800766042/image.png?ex=68bd738e&is=68bc220e&hm=bdb063d4e3a1424f2c5b33aaaff354c9a48abd3a67bc113e4c5cbc90d854a475&=&format=webp&quality=lossless)  |
| Top Plate  | ![top plate](https://media.discordapp.net/attachments/1402478035529240616/1413312856836931594/image.png?ex=68bd73eb&is=68bc226b&hm=01577990e6302ddd55e1969441ed32a88d8dc78ef271a871b7f8f147f7917828&=&format=webp&quality=lossless)  |

## If we had another week
If we had more time, we would expand the project with the following improvements:
* Integrated Perception: Combine color detection, stop sign detection, obstacle avoidance, and lane detection into a single robust pipeline.
* Additional Sensors: Add an extra camera to improve the field of view, and utilize a dedicated power source for the camera and LiDAR to ensure reliability.
* Advanced Behaviors: Design more complex power-ups using color detection, such as a spinning maneuver or other interactive responses.

## Gantt Chart
![Gantt Chart](https://media.discordapp.net/attachments/1402478035529240616/1413301816648798208/Screenshot_2025-09-04_at_3.25.52_PM.png?ex=68bd69a2&is=68bc1822&hm=5bc64b736adb279d846a320cc0386f6fc2c08fae7c0448f1193751189eed0629&=&format=webp&quality=lossless&width=1872&height=520)

## Past Batmobile Accomplishments
* [Donkey Car Autonomous Laps](https://youtu.be/wXakF6iPv3Q?si=_Q1qe9eq6RRz-gBj)
* [Donkey Car GPS Laps](https://youtu.be/d-aGpujyBqg?si=FyLxy5e6jBKazhFP)
* [Lane Detection Laps using ROS2](https://youtu.be/SRFx1kLqEJU?feature=shared)

## Resources
* [Git repository for ucsd_robocar_hub2](https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2)
* [Model for stopsign detector](https://universe.roboflow.com/sign-detection-h24ey/stop-sign-h0vwm)
* [Jetson Nano case/mount](https://cults3d.com/en/3d-model/tool/jetson-nano-case-abs-edition)
* [Final Presentation Slides](https://www.canva.com/design/DAGyDrAzpJw/rflWMh6j-EVrLCU9FfijYw/edit?utm_content=DAGyDrAzpJw&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

## Acknowledgements
Thank you to Alex, Jose, Jack and of course, the rest of the MAE ECE 148 SU25 classmates!

## Contact Information
| Team Member  | Email |
| ------------- | ------------- |
| Karan Humpal  | Content Cell  |
| Burhanuddin Mamujee  | Content Cell  |
| Chenhua(Avery) Wu  | Content Cell  |
| Elizabeth Dam  | edam@ucsd.edu |


![](https://media.discordapp.net/attachments/1402478035529240616/1413304786371215480/UCSDLogo_JSOE_BlueGold.png?ex=68bd6c67&is=68bc1ae7&hm=203cd369ae17e38892ec1d876736cdfebe3b7d790c0d8e63bd2c526212343477&=&format=webp&quality=lossless) 


