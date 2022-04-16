# r2auto_nav
ROS2 code for Tánkyu 2310i <br/>
EG2310 - Fundamentals of Systems Design, AY 21/22, <br/>
Innoovation and Design Programme, National University of Singapore (NUS). <br/>


## Description
In this repository, we use a Robotis Co. Turtlebot3 with ROS2 foxy and Python3 to explore a closed, connected maze with left-wall following algorithm.
Our robot, Tánkyu 2310i, is augmented with LIDAR, NFC-detection, IR-detection and a flywheel firing system to accomplish its set objectives of autonomous navigation, NFC-detection, and firing a ping-pong ball at a target with high IR-signature.

You may check out our [Documentation](https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstley) file for more details on our robot and it's mission.



## File Organisation
- navigationcombine.py file contains the wall-following logic and bug 0 algorithm. This code also communicates with the targeting file to allow the Tánkyu 2310i to engage the target when the conditions are met.
- integration4.py contains the firing algorithm along with all i2c inputs. This code performs all necessary logic processing with the input from the NFC and IR detection systems and communicates the infomation to the navigation code to allow the Tánkyu 2310i to engage the target when the conditions are met.
- Original Files folder is an archive of the forked repository from shihchengyen's r2auto_nav_repository and is not neccesary for the Tankyu 2310i's operations.

## Calibration and configuration
### Navigation Code
Under 'Adjustable variables to calibrate wall follower', you may experiment with different parameters to calibrate the navigtion algorithm to suit your needs.
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| d | Distance from wall| 0.35 |
| fd| Front distance when approaching a merging wall| d + 0.1|
| reverse_d | Distance to reverse when too close to front wall| 0.20|
| speedchange | Linear speed | 0.17|
| turning_speed_wf_fast| Fast rotate speed, used when avoiding obstacle in front| 0.75|
| turning_speed_wf_slow| Slow rotate speed, used when reversing, finding wall or too close to wall| 0.40|
| snaking_radius | Distance from wall before correcting drift | d - 0.07|
| cornering_speed_constant | Coefficient of speedchange during cornering to prevent over/under steer| 0.5|

### Targeting Code
Under 'Adjustable variables to calibrate targeting' you may experiment with different parameters to calibrate the targeting algorithm to suit your needs.
| Variable name| Description| Recommended value  |
| ------------- |:-------------| ---:|
| rotatechange | Angular speed| 0.35 |
| speedchange | Linear speed | 0.20 |
| min_temp_threshold | IR live feed minimum value reference (Appears blue) | 30.0|
| max_temp_threshold | IR live feed maximum value reference (Appears red) | 35.0|
| detecting_threshold | Minimum temperature to identify target as a "Hot target" | 32.0|
| firing_threshold | Acts as a second check after centering target | 35.0|
| ir_offset | Offset angle for each IR sensor orientation (0°→0, 45°→31, 90°→69) | 31.0|

## Operating Instructions
Tested on Ubuntu 20.04 - ROS2 Foxy
### Installation
1. Install Ubuntu 20.04 and ROS 2 Foxy using the instructions in Section 3.1 of the following webpage (Ensure you click on the "Foxy" tab): <br/>
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
2. Using Ubuntu, burn the ROS2 Foxy image to the SD card of the R-Pi’s using the instructions in Section 3.2.3
3. Using Ubuntu, complete the OpenCR setup in Section 3.3
### Set Up
1. Clone this repository to Ubuntu home directory: <br/>
``` 
git clone https://github.com/hahaha2002/r2auto_nav.git 
```
2. Build the package on Ubuntu: <br/>
``` 
cd <path to r2_auto_nav directory>/colcon_ws
colcon build
```
3. Copy the RPi_files folder to the RPi: <br/>
``` 
scp -r <path to r2auto_nav directory>/turtlebot3_ws/src/RPi_files ubuntu@<RPi IP address>:~/turtlebot3/src 
```
4. Build the package on RPi: <br/>
``` 
cd turtlebot3_ws 
colcon build 
```
### Running Instructions
1. Ssh into the RPi, `ssh ubuntu@<IP address of RPi>`
2. On the RPi, Start `rosbu`
3. On Ubuntu, in the r2auto_nav directory, Start the targeting code `python3 integration4.py`.
4. On Ubuntu, in the r2auto_nav directory, Start the navigation code `python3 navigationcombine2.py`.





