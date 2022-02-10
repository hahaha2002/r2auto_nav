import rclpy
from r2mover import Mover

from std_msgs.msg import String
import geometry_msgs.msg


class NFCSubscriber(Mover):

    def __init__(self):
        super().__init__()
        self.subscription = self.create_subscription(
            String,
            'NFC',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        twist = geometry_msgs.msg.Twist()
        if msg.data == "LOADING ZONE":
            # stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    nfc_subscriber = NFCSubscriber()

    rclpy.spin(nfc_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quit()


if __name__ == '__main__':
    main()


