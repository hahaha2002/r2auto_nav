import time

## import thermal sensor requirements
import busio
import board
import adafruit_amg88xx

## import motor requirements
import RPi.GPIO as GPIO

## import NFC requirements
from pn532 import *

## import servo requirements 
import pigpio

## import ros requirements
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

## constants
isDoneLoading = False
isDoneShooting = False
rotatechange = 0.5
speedchange = 0.7
min_temp_threshold = 30.0
max_temp_threshold = 35.0
detecting_threshold = 32.0
firing_threshold = 35.0

## messages sent
NFC_found_msg = 'LOADING ZONE'
load_finish_msg = 'FINISH LOADING'
target_status = 'Not detected'
target_detected_msg = 'Detected'
finish_shooting_msg = "FINISH SHOOTING"

## INITIALISE IR SENSOR
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    amg = adafruit_amg88xx.AMG88XX(i2c)
except:
    print('Please check wiring of IR sensor')

## INITIALISE NFC SENSOR
try:
    pn532 = PN532_I2C(debug=False, reset=20, req=16)
    pn532.SAM_configuration()
except:
    print('Please check wiring of NFC sensor')

## Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

## Set up servo
servo_pin = 12
servo = pigpio.pi()
servo.set_mode(servo_pin, pigpio.OUTPUT)
servo.set_PWM_frequency(servo_pin,50)
servo.set_servo_pulsewidth(servo_pin, 500)

## Set up motor
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) # Ignore warning for now
motor_pin1 = 13
motor_pin2 = 19
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)

## Set up button
button_pin_out = 17
button_pin_in = 27
GPIO.setup(button_pin_out, GPIO.OUT)
GPIO.setup(button_pin_in, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# Set pin 10 to be an input pin and set initial value to be pulled low (off)



class mission(Node):
    def __init__(self):
        super().__init__('mission')

        ## NFC publisher
        self.NFC_publisher_ = self.create_publisher(String, 'NFC', 10)
        timer_period = 0.5
        self.NFC_publish = self.create_timer(timer_period, self.send_nfc_status)

        ## Velocity publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        ## Firing publisher
        self.firing_publisher = self.create_publisher(
            String, 'targeting_status', 10)
        timer_period = 0.5  # seconds
        self.firing_publish = self.create_timer(timer_period, self.send_firing_status)

        ## Lidar subscriber
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile_sensor_data)
        self.lidar_subscription  # prevent unused variable warning

        '''
        ## Odom subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription #prevent unused variable warning
        '''

        ## Position from map subscriber
        self.pos_subscription = self.create_subscription(Pose, '/map2base', self.pos_callback, 10)
        self.pos_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0



    ## Callback functions

    def send_nfc_status(self):
        global NFC_found_msg
        msg = String()
        if not isDoneLoading:
            msg.data = NFC_found_msg
        else:
            msg.data = load_finish_msg
        self.NFC_publisher_.publish(msg)

    def send_firing_status(self):
        global target_status
        msg = String()
        msg.data = target_status
        self.firing_publisher.publish(msg)

    def lidar_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_front = laser_range[0:4]
        np.append(laser_front, laser_range[-1:-4:-1])
        laser_front[laser_front==0] = np.nan
        try:
            self.distance = np.nanmean(laser_front)
        except ValueError:
            self.distance = 9999

    '''
    def odom_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
    '''

    def pos_callback(self, msg):
        global euler_from_quaternion, orientation_quat
        orientation_quat =  msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    ################################################################
    ## Helper functions

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)


    def move_forward(self):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        time.sleep(0.2)
        self.stopbot()


    def euler_from_quaternion(self, x, y, z, w):
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

        return roll_x, pitch_y, yaw_z # in radians


    def rotate(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        rclpy.spin_once(self)
        # get current yaw angle
        current_yaw = self.yaw
        #print(current_yaw)
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        #print(target_yaw)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.vel_publisher.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_di>
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_di>
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)

        self.stopbot()


    def move_servo(self, direction):
        global servo_pin
        duty = int((2000*direction/180)+500)
        servo.set_servo_pulsewidth(servo_pin, duty)
        time.sleep(0.1)


    ################################################################
    ## Phase 1 - Search for NFC

    def nfc_search(self):
        rclpy.spin_once(self)
        loading_bay_found = False
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
            self.send_nfc_status()

    def search(self):
        global isDoneLoading

        # Locate loading bay to retrieve payload
        rclpy.spin_once(self)
        self.nfc_search()

        # Loading balls
        print("loading balls")
        GPIO.output(button_pin_out, 1)
        while not isDoneLoading:
            if GPIO.input(button_pin_in) == GPIO.HIGH:
                isDoneLoading = True
                print("Button is pushed, done loading")
                GPIO.output(button_pin_out, 0)
        self.send_nfc_status()

    #################################################################
    ## Phase 2 - Search and destroy

    def find_target(self):
        # See if target found
        global target_status, target_detected_msg
        target_found = False

        while not target_found:
            for row in amg.pixels:
                for temp in row:
                    if temp > detecting_threshold:
                        target_found = True

        # Target found, communicate with wallfollower to stop working
        target_status = target_detected_msg
        self.send_firing_status()
        self.get_logger().info('Publishing: "%s"' % target_status)
        self.stopbot()
        time.sleep(1)


    def centre_target(self):
        centered = False

        while not centered:
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

            # centre max value between column 3 and 4
            if max_column < 3:
                # spin anti-clockwise
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = rotatechange
                time.sleep(1)
                self.vel_publisher.publish(twist)
                time.sleep(1)
            elif max_column > 4:
                # spin clockwise
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = rotatechange * -1
                time.sleep(1)
                self.vel_publisher.publish(twist)
                time.sleep(1)
            else:
                centered = True
        print('centered')
        self.stopbot()

    def targetting(self):
        global message_sent, servo_pin, motor_pin
        self.rotate_angle = 70
        print('Scanning for hot target...')

        # find the target
        self.find_target() #stopped here

        # centre the target
        self.centre_target()
        time.sleep(1)

        # rotate to forward facing the target
        print('rotating')
        self.rotate(-1) # problems with first time rotating on our robot
        self.rotate(- self.rotate_angle)
        time.sleep(1)
        # move forward until close enough to target
        rclpy.spin_once(self)
        #print(self.distance)
        if self.distance > 0.7:
            while self.distance > 0.7:
                #print(self.distance)
                rclpy.spin_once(self)
                self.move_forward()
            self.stopbot()

            # Recheck center
            print('rotating back')
            self.rotate(self.rotate_angle)
            print('centering again')
            self.centre_target()
            print('rotate again')
            self.rotate(-self.rotate_angle)
        print("Ready to fire!!!")

    def fire(self):
        global target_status, finish_shooting_msg, motor_pin

        # run  motor
        GPIO.output(motor_pin1, 1)
        GPIO.output(motor_pin2, 1)

        # open servo gate
        print("Open servo gate")
        self.move_servo(90)
        time.sleep(5)
        print("Close servo")
        self.move_servo(0)

        # Send message that the target has finished shooting
        self.get_logger().info("Finished shooting")
        target_status = finish_shooting_msg
        self.send_firing_status()
        self.get_logger().info('Publishing: "%s"' % target_status)

        # Stop the DC Motor
        GPIO.output(motor_pin1, 0)
        GPIO.output(motor_pin2, 0)
        self.get_logger().info("Stopped the DC Motor")

        # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")



def main(args=None):
    rclpy.init(args=args)

    MSN = mission()
    MSN.search() #Continuously search for NFC
    MSN.targetting()
    MSN.fire()
    MSN.destroy_node() #Destroy node explicitly, optional otherwise it will be done wh>
    rclpy.shutdown()


if __name__ == '__main__':
    main()












