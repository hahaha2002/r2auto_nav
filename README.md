# r2auto_nav
ROS2 code for Tánkyu 2310i <br/>
EG2310 - Fundamentals of Systems Design, AY 21/22, <br/>
Innoovation and Design Programme, National University of Singapore (NUS). <br/>

--------------------------------------------------------WORK IN PROGRESS--------------------------------------------------------------
## Description
The Tánkyu 2310i, developed by NUS IDP students from EG2310, is a modified Robotis Co. Turtlebot 3. It is augmented with LIDAR, NFC-detection, IR-detection and a flywheel firing system to accomplish its set objectives - Autonomous navigation within a closed and connected maze, locating a loading bay demarketed by NFC tags, and firing a ping-pong ball at a target with high IR-signature.

In this repository, we are using Ubuntu 20.04.4, ROS2 Foxy and Python3.6 to explore a closed and connected maze using a left-wall following algorithm.
You may check out our [Documentation](https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstley) file for more details on our robot and it's mission.

## File Organisation
- [navigation.py](https://github.com/hahaha2002/r2auto_nav/blob/main/navigationcombine.py) code contains the main system and wall-following logic. This code also communicates with the mission code to allow the Tánkyu 2310i to engage on the ir signature when the conditions are met.
- [mission.py](https://github.com/hahaha2002/r2auto_nav/blob/main/mission.py) code initiates all the i2c connections and contain the firing algorithm. This code performs all necessary logic processing with the input from the NFC and IR detection systems and communicates the infomation to the navigation code to allow the Tánkyu 2310i to engage on the ir signature when the conditions are met.
- [Ubuntu_Files](https://github.com/hahaha2002/r2auto_nav/tree/main/Ubuntu_Files) folder is an archive of the miscelleneous code that were tested but not implemented into the final system. The code are functional independently but requires some edits to integrate it into the final system.
- [RPi_Files](https://github.com/hahaha2002/r2auto_nav/tree/main/RPi_Files) folder contains the [factory acceptance test codes](https://github.com/hahaha2002/r2auto_nav/tree/main/RPi_Files/fac_test), all required packages and experiment files for each system. 
- [Original_Files](https://github.com/hahaha2002/r2auto_nav/tree/main/Original_Files) folder is an archive of the forked repository from [shihchengyen's r2auto_nav_repository](https://github.com/shihchengyen/r2auto_nav) and is not neccesary for the Tankyu 2310i's operations.

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

### Mission Code
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
1. Install Ubuntu 20.04 and ROS 2 Foxy using the instructions in Section 3.1 of the [Robotis e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) (Ensure you click on the "Foxy" tab).
For MacBook users, fret not as I have made it through this module with my M1 Macbook Air and have created a [video guide](https://youtu.be/suntoEurFio) to assist you in setting up ubuntu that allows for full functionality with all required applications.
3. Using Ubuntu, complete the RPi in Section 3.2
4. Using Ubuntu, complete the OpenCR setup in Section 3.3 

### Set Up
1. Clone this repository onto your Ubuntu ros2 directory: <br/>
``` 
cd colcon_ws/src/auto_nav
git clone https://github.com/hahaha2002/r2auto_nav.git 
cd colcon_ws
colcon build
```
2. Copy mission.py and the RPi_files folder to the RPi: <br/>
``` 
scp -r <path to r2auto_nav directory>/mission.py ubuntu@<RPi IP address>:~/turtlebot3/src 
scp -r <path to r2auto_nav directory>/RPi_files ubuntu@<RPi IP address>:~/turtlebot3/src 
```
3. Build the package on RPi: <br/>
``` 
cd turtlebot3_ws 
colcon build 
```
4. Install the pigpio package on RPi and automate the daemon on boot:
```
sudo apt install pigpio
sudo systemctl enable pigpiod
```

### Running Instructions
1. Ssh into the RPi, `ssh ubuntu@<RPi IP Address>`
2. On the RPi, initiate the bring up `ros2 launch turtlebot3_bringup robot.launch.py'
3. On RPi, in the RPi_files directory, Start the targeting code `python3 mission.py`.
4. On Ubuntu, in the r2auto_nav directory, Start the navigation code `python3 navigation.py`.





