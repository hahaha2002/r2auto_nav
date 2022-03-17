#! /usr/bin/env python

# import ros stuff
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


import math

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
bug_state_ = 1
bug_state_dict_ = {
    0: 'Turn',
    1: 'Go Straight',
    2: 'Halt',}

# goal
desired_position_ = Point()
desired_position_.x = float(2)
desired_position_.y = float(2)
desired_position_.z = float(0)
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

class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
    def start(self):
        global isArrived
        try:
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
        
        self.publisher_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_:
            #print ('Yaw error: [%s]' % err_yaw)
            self.change_bug_state(1)
    
    def go_straight_ahead(self,des_pos):
        rclpy.spin_once(self)
        global yaw_, pub, yaw_precision_, bug_state_
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
        
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            self.publisher_.publish(twist_msg)
        else:
            #print ('Position error: [%s]' % err_pos)
            self.change_bug_state(2)
            self.stopbot()
        
        # bug_state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            #print ('Yaw error: [%s]' % err_yaw)
            self.change_bug_state(0)
    
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
        self.publisher_.publish(twist)     
        
def main(args=None):
    global pub
    rclpy.init(args=args)
    auto_nav = AutoNav()
    auto_nav.start()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
