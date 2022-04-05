import time
import busio
import board
import adafruit_amg88xx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

## Initialize IR sensor
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    amg = adafruit_amg88xx.AMG88XX(i2c)

except:
    print("Please check wiring of IR sensor")

class IrPublisher(Node):
    def __init__(self):
        super().__init__('ir_publisher')
        ## Create the ir publisher
        self.irpublisher = self.create_publisher(String, 'ir_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.send_matrix)

    def send_matrix(self):
        msg = String()
        msg.data = ''
        for i in range(8):
            msg.data += str(amg.pixels[i]) + ', '
        self.irpublisher.publish(msg)
        self.get_logger().info('Publishing...')

def main(args=None):
    rclpy.init(args=args)
    ir_publisher = IrPublisher()
    rclpy.spin(ir_publisher)
    ir_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




