import RPi.GPIO as GPIO
import rclpy
from pn532 import *
from rclpy.node import Node
from std_msgs.msg import String
GPIO.setwarnings(False)


class NfcPublisher(Node):
    def __init__(self):
        super().__init__('nfc_publisher')
        self.publisher_ = self.create_publisher(String, 'NFC', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = ('LOADING ZONE')
        self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    nfc_publisher = NfcPublisher()
    try:
        pn532 = PN532_I2C(debug=False, reset=20, req=16)
        ic, ver, rev, support = pn532.get_firmware_version()
        print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
        pn532.SAM_configuration()
        print('Waiting for RFID/NFC card...')
        while True:
            # Check if a card is available to read
            uid = pn532.read_passive_target(timeout=0.5)
            print('.', end="")
            # Try again if no card is available.
            if uid is None:
                continue
            print('Found card with UID:', [hex(i) for i in uid])
            rclpy.spin_once(nfc_publisher)
    except Exception as e:
        print(e)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

