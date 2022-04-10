# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
os.system("gnome-terminal --command='ros2 launch turtlebot3_cartographer cartographer.launch.py'")
# open up rslam everytime run

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException
import cv2
import math
import cmath
import time
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats

# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer

# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener


# Adjustable variables to calibrate wall follower
d = 0.45 #Distance from wall
speedchange = 0.15 #Linear speed
back_angles = range(150, 210 + 1, 1)
turning_speed_wf_fast = 0.70  #Fast rotate speed
turning_speed_wf_slow = 0.4  #Slow rotate speed
snaking_radius = d - 0.07  #Amount of variation accepted from wall
cornering_speed_constant = 0.9 #percentage of speed change wwhen cornering

#variables for map file
scanfile = 'lidar.txt'
mapfile = 'map.txt'
myoccdata = np.array([])
occ_bins = [-1, 0, 100, 101]
map_bg_color = 1

#Boolean variables
isTargetDetected = False
isDoneShooting = False
isLoadingBayFound = False
isDoneLoading = False
isMapDone = False

#Waypoint dictionary
waypoint_dict = {
    1: 'nil'
}

#Robot state dictionary
state_ = 0
state_dict_ = {
    0: 'Find the wall',
    1: 'Turn right',
    2: 'Follow the wall',
    3: 'U- Turn',
    4: 'Initial positioning',
    5: 'Reverse',
    'A': 'NFC Found, waiting to receive paylaod',
    'B': 'Payload received, continuing wall-following',
    'C': 'Hot target detected, initiating firing sequence',
    'Cb': 'Hot target detected, waypoint dropped',
    'D': 'Target eliminated',
}

#################
## Bug Neccessities
bugSwitch = True
fd = 0.3

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
bug_state_ = -1
bug_state_dict_ = {
    -1: 'init',
    0: 'Turn',
    1: 'Go Straight',
    2: 'Halt',
}


# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed

dist_precision_ = 0.2

isArrived = False
# publishers
pub = None
#################

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians




class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

        # Create a subscriber
        # Node subscribes to messages from the targeting node
        self.targeting_subscription = self.create_subscription(
            String,
            'targeting_status',
            self.target_callback,
            10)
        self.targeting_subscription  # prevent unused variable warning

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # create subscription to track NFC
        self.nfc_subscription = self.create_subscription(
            String,
            'NFC',
            self.nfc_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        
        #### Bug publisher & subscribers
        self.bug_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bug_odom_subscription = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.bug_scan_subscription = self.create_subscription(LaserScan, 'scan', self.bug_scan_callback, qos_profile_sensor_data)
        self.laser_range = np.array([])
        #self.tfBuffer = tf2_ros.Buffer()
        #self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        ### Map2base requirements
        self.declare_parameter('target_frame', 'base_footprint')
        self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self, spin_thread=True)
        self.mapbase = None
        self.map2base = self.create_publisher(Pose, '/map2base', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def target_callback(self, msg):
        global isTargetDetected, isDoneShooting, waypoint_dict,position
        #self.get_logger().info('In target_callback')
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if (msg.data == 'Detected'):
            isTargetDetected = True
            isDoneShooting = False            
            ###############################################################
            
            #################################################################
            waypoint_dict[2] = position #make RPi send temp, replace 2 with that temp

                                                                        
            self.change_state('C')
        elif (msg.data == 'FINISHED SHOOTING'):
            isDoneShooting = True
            isTargetDetected = False
            self.change_state('D')
            
        else:
            isTargetDetected = False
        

    def odom_callback(self, msg):
        global position
        #self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        position = [round(msg.pose.pose.position.x,1),round(msg.pose.pose.position.y,1),round(msg.pose.pose.position.z,1)]
        #print(self.roll,self.pitch,self.yaw)
        #print(position)

    def occ_callback(self, msg):
        global myoccdata
        #self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(msgdata,occ_bins)
        #calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        # log the info
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            #self.get_logger().info('No transformation found')
            return
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        myoccdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        odata = myoccdata
        np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

    def nfc_callback(self, msg):
        # self.get_logger().info('In nfc_callback')
        global isLoadingBayFound, isDoneLoading
        #Communicates with Targeting code to receive status updates
        if msg.data == 'LOADING ZONE':
            isLoadingBayFound = True
            self.change_state('A')
            
        if msg.data == 'FINISH LOADING':
            isDoneLoading = True
            self.change_state('B')
    
    def timer_callback(self): # for publishing map2base to obtain yaw
        # create numpy array
        msg = Pose()
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
        now = rclpy.time.Time()
        try:
            # while not self.tf_buffer.can_transform(to_frame_rel, from_frame_rel, now, timeout = Durati>
            self.mapbase = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                        # ,
                        # timeout = Duration(seconds=1.0))
        except TransformException as ex:
            # self.get_logger().info(
                # f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        msg.position.x = self.mapbase.transform.translation.x 
        msg.position.y = self.mapbase.transform.translation.y
        msg.orientation = self.mapbase.transform.rotation

        self.map2base.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg)
            
    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        #self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        #self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        #self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * turning_speed_wf_fast
        # start rotation
        self.publisher_.publish(twist)
        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        #self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
    
    #function to print navigation state
    def change_state(self,state):
        global state_, state_dict_
        if state is not state_:
            if type(state) == int:
                print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
            elif type(state) == str:
                print('Targetting - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

    #Main wall-follower logic code ; Left-wall following
    def pick_direction(self):
        #self.get_logger().info('In pick direction:')
        rclpy.spin_once(self)
        self.laserFront = self.laser_range[354:359]
        self.laserFront = np.append(self.laserFront, self.laser_range[0:6])
        self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan = 100)
        self.leftfront_dist = np.nan_to_num(np.nanmean(self.laser_range[40:51]), copy = False, nan = 100)
        self.rightfront_dist = np.nan_to_num(np.nanmean(self.laser_range[310:321]), copy = False, nan = 100)
        self.leftback_dist = np.nan_to_num(np.nanmean(self.laser_range[132:138]), copy = False, nan = 100)
        self.back_dist = np.nan_to_num(np.nanmean(self.laser_range[175:186]), copy = False, nan = 100)
        global d, turning_speed_wf_fast, turningspeed_wf_slow, cornering_speed_constant
        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            if self.leftback_dist > 1.5 * d:
                state_description = 'case X - U-turn'
                self.change_state(3)
                msg.linear.x =  0.6 * speedchange
                msg.angular.z = 1.2 * turning_speed_wf_slow  # turn left to find wall
            else:
                state_description = 'case 1 - nothing'
                self.change_state(0)
                msg.linear.x =  speedchange
                msg.angular.z = turning_speed_wf_slow  # turn left to find wall
                
        elif self.front_dist < 0.25:
            state_description = 'case Y - Reverse!!'
            self.change_state(5)
            msg.linear.x =  -0.7 * speedchange
            msg.angular.z = 0.0
            
                
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            state_description = 'case 2 - front'
            self.change_state(1)
            msg.linear.x = cornering_speed_constant * speedchange
            msg.angular.z = turning_speed_wf_fast

        elif (self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d):
            if (self.leftfront_dist < snaking_radius):
                # Getting too close to the wall
                state_description = 'case 3a - too close to wall'
                self.change_state(1)
                msg.linear.x = speedchange
                msg.angular.z = -turning_speed_wf_slow
            else:
                # Go straight ahead
                state_description = 'case 3b - approaching the wall'
                self.change_state(2)
                msg.linear.x = speedchange

        elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
            state_description = 'case 4  - rfront'
            self.change_state(0)
            msg.linear.x = cornering_speed_constant * speedchange
            msg.angular.z = turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            state_description = 'case 5  - front and rfront'
            self.change_state(1)
            msg.linear.x = cornering_speed_constant * speedchange
            msg.angular.z = -turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            state_description = 'case 6  - lfront and front'
            self.change_state(1)
            msg.angular.z = -turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            state_description = 'case 7  - lfront, front and rfront'
            self.change_state(1)
            msg.linear.x = cornering_speed_constant * 0
            msg.angular.z = -turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            state_description = 'case 8  - lfront and rfront'
            self.change_state(0)
            msg.linear.x = cornering_speed_constant * speedchange
            msg.angular.z = turning_speed_wf_slow  # turn left to find wall
    
        else:
            state_description = 'unknown case'
            print('Unkown case')
            pass
        # Send velocity command to the robot
        self.publisher_.publish(msg)

    #Function to stop bot
    def stopbot(self):
        #self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    #Function for bot to locate first wall    
    def initialmove(self):
        #find nearest wall
        #split lidar into 4 regions
        global d
        self.change_state(4)
        twist = Twist()
        self.laserFront = self.laser_range[315:359]
        self.laserFront = np.append(self.laserFront, self.laser_range[0:46])
        self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan=100)
        self.rear_dist = np.nan_to_num(np.nanmean(self.laser_range[135:226]), copy = False, nan = 100)
        self.left_dist = np.nan_to_num(np.nanmean(self.laser_range[45:136]), copy = False, nan = 100)
        self.right_dist = np.nan_to_num(np.nanmean(self.laser_range[226:316]), copy = False, nan = 100)
        self.laser_regions = [self.front_dist,self.left_dist,self.rear_dist,self.right_dist]
        self.closest_wall = self.laser_regions.index(min(self.laser_regions))
        self.rotatebot(90*self.closest_wall)
        while self.front_dist > d:    
            self.laserFront = self.laser_range[315:359]
            self.laserFront = np.append(self.laserFront, self.laser_range[0:46])
            self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan=100)
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
        self.stopbot()
        self.rotatebot(-45)
        
        
#main navigation code
    def mover(self):
        global myoccdata, isTargetDetected, isDoneShooting, isLoadingBayFound, isDoneLoading, position, d
        try:
            rclpy.spin_once(self)
            # ensure that we have a valid lidar data before we start wall follow logic
            print("Acquiring lidar data")
            while (self.laser_range.size == 0):    
                rclpy.spin_once(self)
            # initial move to find the appropriate wall to follow
            self.initialmove()
            
            initial_time = time.time()
            start_time = initial_time + 10 # to ensure start point is accessible afterwards
            start_position = []
            while rclpy.ok():
                rclpy.spin_once(self)
                if self.laser_range.size != 0:
                    #Increases distance from wall if NFC still not detected after one round
                    if int(time.time()) == int(start_time):
                        start_position = position
                        print('Starting point: ',start_position)
                    if position == start_position and (time.time()-start_time > 60):
                        d = d+0.05
                        start_time = time.time()
                        start_position = position
                        print("Returned to start point without NFC, distance increased by 7cm")
                    
                    # while there is no target detected, keep picking direction (do wall follow)
                    if not isTargetDetected :
                        self.pick_direction()
                    # Halt until signal received from targetting code
                    if isLoadingBayFound:
                        while not isDoneLoading:
                            self.stopbot()
                            rclpy.spin_once(self)
                    # Halt wall following and allow targetting code to engage the target
                    if isTargetDetected and isMapDone:
                        self.stopbot()
                        while (not isDoneShooting):
                            #print('Target detected, initiating firing sequence!')
                            rclpy.spin_once(self)
                        isTargetDetected = False
                # allow the callback functions to run
                rclpy.spin_once(self)
              
        #except Exception as e:
           # print(e)
 
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
            #cv2.imwrite('mazemapfinally.png', myoccdata)
  
    ############################################################################3BUG STUFF#########################################################      
    ### Bug codes
    
    def bug_scan_callback(self, msg):
        global regions_
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan
        self.laserFront = self.laser_range[354:359]
        self.laserFront = np.append(self.laserFront, self.laser_range[0:6])
        self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan = 100)
        self.leftfront_dist = np.nan_to_num(np.nanmean(self.laser_range[43:48]), copy = False, nan = 100)
        self.rightfront_dist = np.nan_to_num(np.nanmean(self.laser_range[313:318]), copy = False, nan = 100)
        self.leftback_dist = np.nan_to_num(np.nanmean(self.laser_range[132:138]), copy = False, nan = 100)
        self.back_dist = np.nan_to_num(np.nanmean(self.laser_range[175:186]), copy = False, nan = 100)
        
        regions_ = {
        'right':  min(min(self.laser_range[265:276]), 10),
        'fright': min(self.rightfront_dist, 10),
        'front':  min(self.front_dist, 10),
        'fleft':  min(self.leftfront_dist, 10),
        'left':   min(min(msg.ranges[85:96]), 10),
        }

    def getTarget(self, x_coord,y_coord,z_coord):
        self.desired_position_ = Point()
        self.desired_position_.x = x_coord
        self.desired_position_.y = y_coord
        self.desired_position_.z = z_coord
  
    def start(self):
        global isArrived, waypoint_dict
        try:
            rclpy.spin_once(self)
            print("Acquiring lidar data")
            while (self.laser_range.size == 0):    
                rclpy.spin_once(self)
            
            while not isArrived:
                if bug_state_ == 0:
                    self.fix_yaw(self.desired_position_)
                elif bug_state_ == 1:
                    self.go_straight_ahead(self.desired_position_)
                elif bug_state_ == 2:
                    self.stopbot()
                    isArrived = True
                    pass
                elif bug_state_ == -1:
                    self.change_bug_state(0)
                else:
                    print('Unknown bug state!')
                    pass
        except Exception as e:
            print(e)
        # Ctrl-c detected
        finally:
            # stop moving
            print(waypoint_dict)
            self.stopbot()
            
    def clbk_odom(self, msg):
        global position_
        global yaw_
        # position
        position_ = msg.pose.pose.position
        #print(position_)
        # yaw
        euler = euler_from_quaternion(msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        yaw_ = euler[2]
            
    def fix_yaw(self,des_pos):
        global yaw_, pub, yaw_precision_, bug_state_
        rclpy.spin_once(self)
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_:
            #turn left if deired yaw -ve
            twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
        
        self.bug_publisher_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_:
            #print ('Yaw error: [%s]' % err_yaw)
            self.change_bug_state(1)
    
    def go_straight_ahead(self,des_pos):
        global yaw_, pub, yaw_precision_, bug_state_, fd, bugSwitch
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
        if err_pos > dist_precision_ and bugSwitch: 
            rclpy.spin_once(self)
            if self.front_dist > fd:
                twist_msg = Twist()
                twist_msg.linear.x = 0.2
                self.bug_publisher_.publish(twist_msg)
            elif self.front_dist < fd:
                self.stopbot()
                self.change_bug_switch(0)
                self.bugWall()
                
        # bug_state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            #print ('Yaw error: [%s]' % err_yaw)
            self.change_bug_state(0)
        if err_pos < dist_precision_:
            self.stopbot()
            self.change_bug_state(2)
 
    

    def normalize_angle(self,angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    
    def check_whether_to_switch(self):
            rclpy.spin_once(self)
            desired_yaw = math.atan2(self.desired_position_.y - position_.y, self.desired_position_.x - position_.x)
            err_yaw = self.normalize_angle(desired_yaw - yaw_)
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
               regions_['front'] > 1.5 and regions_['fright'] > 1 and regions_['fleft'] > 1:
               #print('less than 30')
               return 1            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['left'] > 1.5 and regions_['fleft'] > 1:
               #print('between 30 and 90 - to the left')        
               return 1           
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['right'] > 1.5 and regions_['fright'] > 1:
                #print('between 30 and 90 - to the right')    
                return 1
            return 0
            
    def change_bug_switch(self,switch):
        global bugSwitch
        if switch == 0:
            bugSwitch = False
        elif switch ==1:
            bugSwitch = True
    
    def change_bug_state(self,bug_state):
        global bug_state_
        if bug_state is not bug_state_:
            print('Bug algorithm - [%s] - %s' % (bug_state, bug_state_dict_[bug_state]))
        bug_state_ = bug_state
    
    
    def bugAlgo(self):
        '''
        des_waypoint = (float(waypoint_dict[max(waypoint_dict)][0]),
                       float(waypoint_dict[max(waypoint_dict)][1]),
                       float(waypoint_dict[max(waypoint_dict)][2]))
        getTarget(des_waypoint)
        '''
        self.getTarget(0.5,0.0,0.0)
        self.start()
    
    def bugWall(self):
        checkVal = self.check_whether_to_switch()
        while not checkVal:
            self.pick_direction()
            checkVal = self.check_whether_to_switch()
        print('Bug algorithm - [%s] - %s' %(3,'Switch'))
        self.change_bug_switch(1)


def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNav()
    auto_nav.mover()
    #auto_nav.bugAlgo()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
