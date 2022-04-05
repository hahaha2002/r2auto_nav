import RPi.GPIO as GPIO
import rclpy
from pn532 import *
GPIO.setwarnings(False)


def main(args=None):
    rclpy.init(args=args)
    try:
        ## Initialise NFC sensor
        pn532 = PN532_I2C(debug=False, reset=20, req=16)
        pn532.SAM_configuration()
        print('Waiting for RFID/NFC card...')
        while True:
            # Check if a card is available to read
            uid = pn532.read_passive_target(timeout=0.5)
            # Try again if no card is available.
            if uid is None:
                continue
            # Print if card is detected
            print('NFC Detected')

    except KeyboardInterrupt:
        print('Keyboard interrupt')

    except:
        print('Please check wiring of NFC sensor')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

