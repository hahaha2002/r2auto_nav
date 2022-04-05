from servo_functions import move_servo
import time
import rclpy
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
distance = 9999
current_direction = 90.000000

class Servo_Subscriber(Node):
    def __init__(self):
        super().__init__('servo_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global current_direction
        laser_range = np.array(msg.ranges)
        laser_front = laser_range[0:5]
        np.append(laser_front, laser_range[-5:-1])
        laser_front[laser_front==0] = np.nan
        try:
            distance = np.nanmean(laser_front)
        except ValueError:
            distance = 9999
        print('distance %s' % distance)
        print('direction %f' % current_direction)
        if float(distance) <= 1 and current_direction != 0.000000:
            move_servo(0)
            time.sleep(1)
            current_direction = 0.000000
        elif float(distance) > 1 and current_direction != 90.000000:
            move_servo(90)
            time.sleep(1)
            current_direction = 90.000000

def main(args=None):
    rclpy.init(args=args)
    servo_subscriber = Servo_Subscriber()
    rclpy.spin(servo_subscriber)
    servo_subscriber.destroy_node()
    rclpy.shutdown()
    exec(open('servo_functions.py').read())

if __name__ == '__main__':
    main()
