#from converter import format_array
import time
import busio
import board
import adafruit_amg88xx

import RPi.GPIO as GPIO
from pn532 import *

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist as Twist

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
import math
import cmath
#from scipy import interpolate

#CONSTANTS
min_temp_threshold = 15
max_temp_threshold = 35

#INITIALISE IR SENSOR
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    amg = adafruit_amg88xx.AMG88XX(i2c)
except:
    print('Please check wiring of IR sensor')

#INITIALISE NFC SENSOR
try:
    pn532 = PN532_I2C(debug=False, reset=20, req=16)
    pn532.SAM_configuration()
except:
    print('Please check wiring of NFC sensor')

#INITIATE ALL PUBLISHERS AND SUBSCRIBERS
class IrPublisher(Node):
    def __init__(self):
        super().__init__('ir_publisher')
        self.publisher_ = self.create_publisher(String, 'ir_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.send_matrix)
    def send_matrix(self):
        msg = String()
        msg.data = ''
        for i in range(8):
            msg.data += str(amg.pixels[i]) + ', '
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')

class IrStatusPublisher(Node):
    def __init__(self):
        super().__init__('ir_status_publisher')
        self.publisher_ = self.create_publisher(String, 'ir_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.send_ir_status)
    def send_ir_status(self):
        msg = String()
        msg.data = 'TARGET SPOTTED'
        self.publisher_.publish(msg)

class NfcPublisher(Node):
    def __init__(self):
        super().__init__('nfc_publisher')
        self.publisher_ = self.create_publisher(String, 'NFC', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.send_nfc_status)
    def send_nfc_status(self):
        msg = String()
        msg.data = 'LOADING ZONE'
        self.publisher_.publish(msg)

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.distance_at_90_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def distance_at_90_callback(self,msg):
        global laser_90
        laser_range = np.array(msg.ranges)
        laser_90 = np.nanmean(laser_range[87:94])

#MAIN LOGIC CODE
class searchAndDestroy(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        '''
        # Set up publisher 'targeting_status' to communicate with wallfollower
        self.publisher_targeting = self.create_publisher(
            String, 'targeting_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        '''
    def nfc_search(self):
        nfc_publisher = NfcPublisher()
############################################################################
############################################################################
############################################################################
############################################################################
        loading_bay_found = False
############################################################################
############################################################################
############################################################################
############################################################################
        print('Searching for loading bay...')
        while not loading_bay_found:
            # Check if a card is available to read
            nfc_reading = pn532.read_passive_target(timeout=0.5)
            print('.', end="")
            # Try again if no card is available.
            if nfc_reading is None:
                continue
            print('Arrived at loading bay!')
            loading_bay_found = True
            rclpy.spin_once(nfc_publisher)

    def target_search(self):
        ir_publisher = IrPublisher()
        ir_status_publisher = IrStatusPublisher()
        target_found = False
        print('Scanning for target...')
        while not target_found:
            detected = False
            rclpy.spin_once(ir_publisher)
            for row in amg.pixels:
                for temp in row:
                    if temp > max_temp_threshold:
                        target_found = True
        # Communicate with wallfollower to set waypoint / stop
        #rclpy.spin_once(ir_status_publisher)
        self.target_close_in()

    def target_close_in(self):
        twist = Twist()
        lidar_subscriber = LidarSubscriber()
        rclpy.spin_once(lidar_subscriber)
        if laser_90 > 100: # if lidar distance at 90 degrees more than effectiverange,
           #rotate bot 90 degrees
           #move forward till laser_90<100
           #rotate bot -90 degrees
        #else:
            pass
        self.target_zeroing()

    def target_zeroing(self):
        ir_publisher = IrPublisher()
        rotatechange = 0.1
        speedchange = 0.05
        horizontally_centered = False
        twist = Twist()
        while not horizontally_centered:
            rclpy.spin_once(ir_publisher)
            screen = amg.pixels
            max_row = 0
            max_column = 0
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(screen[row])):
                    current_value = screen[row][column]
                    if current_value > max_value:
                        max_row = row
                        max_column = column
                        max_value = current_value

            if not horizontally_centered:
                print('auto zeroing in progress...')
                # centre max value between row 3 and 4
                if max_column < 3:
                    # spin it anti-clockwise
                    twist.linear.x = 0.0
                    twist.angular.z = rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                elif max_column > 4:
                    # spin it clockwise
                    twist.linear.x = 0.0
                    twist.angular.z = -1 * rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                else:
                    horizontally_centered = True
                    print('target zeroed')
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    time.sleep(1)
            rclpy.spin_once(ir_publisher)

    def search(self):
        #locate loading bay to retrieve payload
        self.nfc_search()
        #locate target to destroy
        self.target_search()
        self.destroy()

    def destroy(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    SnD = searchAndDestroy()
    SnD.search() #Continuously search for NFC
    SnD.destroy_node() #Destroy node explicitly, optional otherwise it will be done when garbage collector destroys the node object
    rclpy.shutdown()


if __name__ == '__main__':
    main()

