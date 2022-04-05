import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

motor_pin = 12
GPIO.setup(motor_pin, GPIO.OUT)

while True:
        try:
                cmd = input("Enter command (1/0):")
                if cmd == '1':
                        GPIO.output(motor_pin, 1)
                elif cmd == '0':
                        GPIO.output(motor_pin, 0)
                else:
                        print("Invalid command")

        except KeyboardInterrupt:
                GPIO.cleanup()
                exit()

