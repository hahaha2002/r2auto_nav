#! /usr/bin/env python

# import ros stuff
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import time
import numpy as np

import math
import navigation as nav



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
    2: 'Halt',}

# goal
'''
desired_position_ = Point()
desired_position_.x = float(0)
desired_position_.y = float(-3.7)
desired_position_.z = float(0)
'''
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed

dist_precision_ = 0.2

isArrived = False
# publishers
pub = None

def euler_from_quaternion(x, y, z, w):
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

    return roll_x, pitch_y, yaw_z,# in radians

def getTarget(x_coord,y_coord,z_coord):
    global desired_position_
    desired_position_ = Point()
    desired_position_.x = x_coord
    desired_position_.y = y_coord
    desired_position_.z = z_coord

class BugNav(Node):
    def __init__(self):
        super().__init__('bug_nav')
        self.bug_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bug_odom_subscription = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.bug_scan_subscription = self.create_subscription(LaserScan, 'scan', self.bug_scan_callback, qos_profile_sensor_data)
        self.laser_range = np.array([])
        #self.tfBuffer = tf2_ros.Buffer()
        #self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
    
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

  
    def start(self):
        global isArrived
        try:
            rclpy.spin_once(self)
            print("Acquiring lidar data")
            while (self.laser_range.size == 0):    
                rclpy.spin_once(self)
            
            while not isArrived:
                if bug_state_ == 0:
                    self.fix_yaw(desired_position_)
                elif bug_state_ == 1:
                    self.go_straight_ahead(desired_position_)
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
                print('Time to change to wall')
                self.change_bug_switch(0)
                nav.AutoNav().bugWall()
                

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
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = self.normalize_angle(desired_yaw - yaw_)
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
               regions_['front'] > 1.5 and regions_['fright'] > 1 and regions_['fleft'] > 1:
                print('less than 30')
                return 1            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['left'] > 1.5 and regions_['fleft'] > 1:
                print('between 30 and 90 - to the left')
                return 1           
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['right'] > 1.5 and regions_['fright'] > 1:
                print('between 30 and 90 - to the right')
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
        
    def stopbot(self):
        #self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.bug_publisher_.publish(twist)     


'''
def main(args=None):
    global pub
    #rclpy.init(args=args)
    bug_nav = BugNav()
    bug_nav.start()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bug_nav.destroy_node()
    #ssrclpy.shutdown()

if __name__ == '__main__':
    main()
'''