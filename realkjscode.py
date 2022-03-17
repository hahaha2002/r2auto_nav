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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import math
import cmath
import time
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats

# constants
d = 0.4
rotatechange = 1.4
speedchange = 0.3
back_angles = range(150, 210 + 1, 1)

scanfile = 'lidar.txt'
mapfile = 'map.txt'
myoccdata = np.array([])
occ_bins = [-1, 0, 100, 101]
map_bg_color = 1
isTargetDetected = False
isDoneShooting = False
isLoadingBayFound = False
isDoneLoading = False
initial_direction = "Back"

state_ = 0
state_dict_ = {
    0: 'Find the wall',
    1: 'Turn right',
    2: 'Follow the wall',
    3: 'U- Turn',
    4: 'Initial positioning'
}

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



        # Create a subscriber
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /en613/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/state_est',
            self.state_estimate_callback,
            10)
        self.subscription  # prevent unused variable warning



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



    def state_estimate_callback(self, msg):
        """
        Extract the position and orientation data.
        This callback is called each time
        a new message is received on the '/en613/state_est' topic
        """
        # Update the current estimated state in the global reference frame
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

    def target_callback(self, msg):
        global isTargetDetected, isDoneShooting
        #self.get_logger().info('In target_callback')
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if (msg.data == 'Detected'):
            print('Target Detected')
            isTargetDetected = True
            isDoneShooting = False
        elif (msg.data == 'Done'):
            print('Is Done shooting')
            isDoneShooting = True
            isTargetDetected = False
        else:
            print('No Target Detected')
            isTargetDetected = False
        

    def odom_callback(self, msg):
        global position
        #self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        position = [round(msg.pose.pose.position.x,1),round(msg.pose.pose.position.y,1),round(msg.pose.pose.position.z,1)]
       #print(self.roll,self.pitch,self.yaw)


















    def occ_callback(self, msg):
        global myoccdata
        self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(msgdata,occ_bins)
        #calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        # log the info
        self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
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
        global isLoadingBayFound, isDoneLoading
        #self.get_logger().info('In NFC callback')
        #self.get_logger().info('I heard: "%s" ' % msg.data)
        if msg.data == 'LOADING ZONE':
            isLoadingBayFound = True
            print("NFC FOUND")
            print("Halting to receive payload...")
        if msg.data == 'FINISH LOADING':
            isDoneLoading = True
            print('Payload Received!')
            
            
    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
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
        twist.angular.z = c_change_dir * rotatechange
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
        
    def change_state(self,state):
        global state_, state_dict_
        if state is not state_:
            print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

    def pick_direction(self):
        #self.get_logger().info('In pick direction:')
        self.laserFront = self.laser_range[354:359]
        self.laserFront = np.append(self.laserFront, self.laser_range[0:6])
        self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan=100)
        self.leftfront_dist = np.nan_to_num(np.nanmean(self.laser_range[43:48]), copy = False, nan = 100)
        self.rightfront_dist = np.nan_to_num(np.nanmean(self.laser_range[313:318]), copy = False, nan = 100)
        self.leftback_dist = np.nan_to_num(np.nanmean(self.laser_range[132:138]), copy = False, nan = 100)
        #self.get_logger().info('Front Distance: %s' % str(self.front_dist))
        #self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
        #self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))
        global d
        # wall distance from the robot. It will follow the right wall and maintain this distance
        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.3  # Fast turn ideal = 1.0
        self.turning_speed_wf_slow = 0.5  # Slow turn = 0.50
        self.snaking_radius = d - 0.05
        # Set movement speed
        self.forward_speed = speedchange
        self.cornering_speed_constant = 0.2
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
                msg.linear.x =  0.6 * self.forward_speed
                msg.angular.z = 1.2 * self.turning_speed_wf_slow  # turn left to find wall
            else:
                state_description = 'case 1 - nothing'
                self.change_state(0)
                msg.linear.x =  self.forward_speed
                msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            state_description = 'case 2 - front'
            self.change_state(1)
            msg.linear.x = self.cornering_speed_constant * self.forward_speed
            msg.angular.z = -self.turning_speed_wf_fast

        elif (self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d):
            if (self.leftfront_dist < self.snaking_radius):
                # Getting too close to the wall
                state_description = 'case 3a - too close to wall'
                self.change_state(1)
                msg.linear.x = self.forward_speed
                msg.angular.z = -self.turning_speed_wf_slow
            else:
                # Go straight ahead
                state_description = 'case 3b - approaching the wall'
                self.change_state(2)
                msg.linear.x = self.forward_speed

        elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
            state_description = 'case 4  - rfront'
            self.change_state(0)
            msg.linear.x = self.cornering_speed_constant * self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            state_description = 'case 5  - front and rfront'
            self.change_state(1)
            msg.linear.x = self.cornering_speed_constant * self.forward_speed
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            state_description = 'case 6  - lfront and front'
            self.change_state(1)
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            state_description = 'case 7  - lfront, front and rfront'
            self.change_state(1)
            msg.linear.x = self.cornering_speed_constant * self.forward_speed
            msg.angular.z = -self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            state_description = 'case 8  - lfront and rfront'
            self.change_state(0)
            msg.linear.x = self.cornering_speed_constant * self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow  # turn left to find wall
        else:
            state_description = 'unknown case'
            print('Unkown case')
            pass
        # Send velocity command to the robot
        self.publisher_.publish(msg)


    def stopbot(self):
        #self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    '''
    def initialmove(self):
        #find nearest wall
        #split lidar into 4 regions
        twist = Twist()
        self.laserFront = self.laser_range[315:359]
        self.laserFront = np.append(self.laserFront, self.laser_range[0:46])
        self.front_dist = np.nan_to_num(np.nanmean(self.laserFront), copy = False, nan=100)
        self.rear_dist = np.nan_to_num(np.nanmean(self.laser_range[135:226]), copy = False, nan = 100)
        self.left_dist = np.nan_to_num(np.nanmean(self.laser_range[45:136]), copy = False, nan = 100)
        self.right_dist = np.nan_to_num(np.nanmean(self.laser_range[226:316]), copy = False, nan = 100)
        self.laser_regions = [self.front_dist,self.left_dist,self.rear_dist,self.right_dist]
        self.closest_wall = self.laser_regions.index(min(self.laser_regions))
        print( self.closest_wall)
        if self.laser_regions[self.closest_wall] > d:
            self.rotatebot(90* self.closest_wall)
        lrback = ( self.front_dist < float(
            d -0.05)).nonzero()
        self.publisher_.publish(twist)
        while len(lrback[0]) <= 0:
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            rclpy.spin_once(self)
            lrback = (self.laser_range[back_angles] < float(
                d-0.05)).nonzero()
            self.publisher_.publish(twist)
        self.stopbot()
        self.rotatebot(90)
        self.stopbot()
    '''
    def initialmove(self):
        self.get_logger().info('In initialmove, move backwards')
        # publish to cmd_vel to move TurtleBot
        if initial_direction == "Back":
            self.get_logger().info("Going back")
        elif initial_direction == "Right":
            self.get_logger().info("Going right")
            self.rotatebot(90)
        elif initial_direction == "Left":
            self.get_logger().info("Going right")
            self.rotatebot(-90)
        elif initial_direction == "Front":
            self.get_logger().info("Going Forward")
            self.rotatebot(180)
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        lrback = (self.laser_range[back_angles] < float(
            0.40)).nonzero()
        self.publisher_.publish(twist)
        while len(lrback[0]) <= 0:
            time.sleep(1)
            twist.linear.x = -speedchange
            twist.angular.z = 0.0
            rclpy.spin_once(self)
            lrback = (self.laser_range[back_angles] < float(
                0.40)).nonzero()
            self.publisher_.publish(twist)
        self.stopbot()
        self.rotatebot(-90)
        self.stopbot()

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
            start_time = initial_time + 7
            start_position = []
            while rclpy.ok():
                if int(time.time()) == int(start_time):
                    self.stopbot()
                    time.sleep(2)
                    start_position = position
                    print('Starting point: ',start_position)
                if position == start_position and (time.time()-start_time > 8):
                    d = d+0.07
                    start_time = time.time()
                    start_position = position
                    print("Returned to start point without NFC, distance increased by 10cm")
                        
                if self.laser_range.size != 0:
                    if isLoadingBayFound:
                        while not isDoneLoading:
                            self.stopbot()
                            print('Awaiting payload...')
                            rclpy.spin_once(self)
                    #if not isDoneLoading:
                     #   rclpy.spin_once(self)
                    # if NFC loading zone is detected, stop the bot until button pressed
                    
                    # while there is no target detected, keep picking direction (do wall follow)
                    if not isTargetDetected:
                        self.pick_direction()
                    
                    # when there is target detected, stop the bot and stop wall following logic
                    # until it finish shooting at the target.
                    # Then set isTargetDetected to False to resume the wall following logic

                    else:
                        self.stopbot()
                        while (not isDoneShooting):
                            print('In mover, target detected.')
                            rclpy.spin_once(self)
                        isTargetDetected = False

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
            #cv2.imwrite('mazemapfinally.png', myoccdata)



def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
