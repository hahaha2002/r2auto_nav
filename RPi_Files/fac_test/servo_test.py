import pigpio

servo_pin = 14
servo = pigpio.pi()
servo.set_mode(servo_pin, pigpio.OUTPUT)
servo.set_PWM_frequency(servo_pin,50)
servo.set_servo_pulsewidth(servo_pin, 500)

def move_servo():
    direction = int(input('Enter angle for servo rotation: '))
    duty = int((2000*direction/180)+500)
    servo.set_servo_pulsewidth(servo_pin, duty)

while True:
    try:
        move_servo()
    except Exception as e:
        print(e)
