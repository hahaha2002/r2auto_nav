import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, String
import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO
import pigpio

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
import cmath
import numpy as np

# constants
rotatechange = 0.2
speedchange = 0.05
detecting_threshold = 32.0
firing_threshold = 35.0

isDoneShooting = False

message_sent = 'Not Detected'

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

# Set up servo
servo_pin = 19
servo = pigpio.pi()
servo.set_mode(servo_pin, pigpio.OUTPUT)
servo.set_PWM_frequency(servo_pin,50)
servo.set_servo_pulsewidth(servo_pin, 500)

# Set up motor
GPIO.setmode(GPIO.BCM)
motor_pin = 12
GPIO.setup(motor_pin, GPIO.OUT)

# Messages


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

    return roll_x, pitch_y, yaw_z # in radians


class ThermalCamera(Node):
    def __init__(self):
        super().__init__('thermalcamera')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Set up publisher 'targeting_status' to communicate with wallfollower
        self.publisher_targeting = self.create_publisher(
            String, 'targeting_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.distance = 999
        self.lidarsubscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.lidarsubscription  # prevent unused variable warning
        self.odomsubscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odomsubscription #prevent unused variable warning
        self.roll, self.pitch, self.yaw = 0,0,0

    def odom_callback(self, msg):
        # self.get_logger().info(msg)
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def listener_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_front = laser_range[0:4]
        np.append(laser_front, laser_range[-1:-4:-1])
        laser_front[laser_front==0] = np.nan
        try:
            self.distance = np.nanmean(laser_front)
        except ValueError:
            self.distance = 9999


    # targeting_status callback function to stop wallfollower logic when target is detected
    def timer_callback(self):
        global message_sent
        msg = String()
        msg.data = message_sent
        self.publisher_targeting.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def find_target(self):

        # See if target found
        global message_sent
        target_found = False

        # -------------------------------------------------------------------------- #
        # While target has not been found, run this code to check if target is found #
        # -------------------------------------------------------------------------- #
        while not target_found:
            detected = False

            for row in amg.pixels:
                #print('[', end=" ")
                for temp in row:
                    if temp > detecting_threshold:
                        detected = True
                        target_found = True
                '''
                    print("{0:.1f}".format(temp), end=" ")
                print("]")
                print("\n")
                '''

            if detected == True:
                # Communicate with wallfollower to stop working
                message_sent = 'Detected'
                self.timer_callback()
                print(" ")
                print("DETECTED!!")
                print("]")
                print("\n")
                time.sleep(1)

        # If target is found, stop movement
        self.stopbot()

        return True

    def centre_target(self):
        # ----------------------------------------------------------- #
        # Adjust the servo and robot until high temp is in the centre #
        # ----------------------------------------------------------- #

        # Centre the target in the robot's vision
        GPIO.setmode(GPIO.BCM)
        horizontally_centered = False
        vertically_centered = True #change back to False
        centered = False
        global servo_pin

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

            if not horizontally_centered:
                # centre max value between column 3 and 4
                if max_column < 3:
                    # spin anti-clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                elif max_column > 4:
                    # spin clockwise
                    twist = Twist()
                    twist.linear.x = speedchange
                    twist.angular.z = rotatechange * -1
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                else:
                    horizontally_centered = True
                print('centered')
                self.stopbot()

            if horizontally_centered and not vertically_centered:
                # centre max value between row 3 and 4
                if max_row < 3:
                    # shift the servo up by 5 degrees (limit:0)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 0

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                elif max_row > 4:
                    # shift the servo down by 5 degrees (limit: 20)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 20

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                else:
                    vertically_centered = True

            if horizontally_centered and vertically_centered:
                centered = True

            return True

    def rotate(self, rot_angle):
        print('rotate')
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        rclpy.spin_once(self)
        # get current yaw angle
        current_yaw = self.yaw
        print(current_yaw)
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        print(target_yaw)
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
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)

        self.stopbot()

    def move_to_target(self):
        # move to the object in increments
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(0.2)
        self.stopbot()

    def firing_time(self):
        screen = amg.pixels
        for row in [3, 4]:
            for column in [3, 4]:
                if screen[row][column] > firing_threshold:
                    return True

    def spool_motor(self):
        pass

    def move_servo(self, direction):
        global servo_pin
        duty = int((2000*direction/180)+500)
        servo.set_servo_pulsewidth(servo_pin, duty)
        time.sleep(0.1)

    def targetting(self):
        global message_sent, servo_pin, motor_pin

        # find the target
        self.find_target()

        # centre the target
        self.centre_target()
        time.sleep(1)

        # --------------------------------------------------- #
        # Now it is centered, start moving towards the target #
        # --------------------------------------------------- #
        self.rotate(1)
        self.rotate(-80)
        time.sleep(1)
        while self.distance > 0.4:
            rclpy.spin_once(self)
            print(self.distance)
            self.move_to_target()
        self.stopbot()

        # Recheck center
        self.rotate(80)
        self.centre_target()
        self.rotate(-80)

        # ----------------------------- #
        # Now the bot can fire the ball #
        # ----------------------------- #

        # run  motor
        GPIO.output(motor_pin, 1)

        '''
        GPIO.setmode(GPIO.BCM)

        # 11 , 13 Input 1,2
        GPIO.setup(17, GPIO.OUT)
        GPIO.setup(27, GPIO.OUT)
        self.get_logger().info("Setup the DC")

        # 12 Enable
        GPIO.setup(18, GPIO.OUT)
        pwm = GPIO.PWM(18, 100)
        pwm.start(0)
        time.sleep(5)

        # Spin Backwards Continuously
        GPIO.output(17, True)
        GPIO.output(27, False)
        pwm.ChangeDutyCycle(75)
        GPIO.output(18, True)
        self.get_logger().info("Start the DC")

        # Start the Stepper Motor 2 seconds later

        # Wait for 2 seconds
        time.sleep(5)

        # Set up the Stepper Pins
        control_pins = [26, 19, 13, 6]
        for pin in control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        halfstep_seq = [
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
            [1, 0, 0, 1]]

        self.get_logger().info("Started the Stepper")
        # Start Spinning the Stepper
        for i in range(512):
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
                time.sleep(0.001)
        '''

        #open servo gate
        self.move_servo(90)
        time.sleep(5)
        self.move_servo(0)

        # -------------- #
        # Do the cleanup #
        # -------------- #
        # Send message that the target has finished shooting
        self.get_logger().info("Finished shooting")
        message_sent = 'FINISHED SHOOTING'
        self.timer_callback()

        # Stop the DC Motor
        GPIO.output(motor_pin, 0)
        self.get_logger().info("Stopped the DC Motor")

        # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)

    thermalcamera = ThermalCamera()
    thermalcamera.targetting()
    #thermalcamera.rotate(90) #test
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermalcamera.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
