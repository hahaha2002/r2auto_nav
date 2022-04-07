import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

motor_pin1 = 13
motor_pin2 = 19

GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)

while True:
        try:
                cmd = input("Enter command (1/0):")
                if cmd == '1':
                        GPIO.output(motor_pin1, 1)
                        GPIO.output(motor_pin2, 1)
                elif cmd == '0':
                        GPIO.output(motor_pin1, 0)
                        GPIO.output(motor_pin2, 0)
                else:
                        print("Invalid command")

        except KeyboardInterrupt:
                GPIO.cleanup()
                exit()

