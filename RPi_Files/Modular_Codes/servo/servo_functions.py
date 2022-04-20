
import time
import pigpio

servo_pin = 19
servo = pigpio.pi()
servo.set_mode(servo_pin, pigpio.OUTPUT)
servo.set_PWM_frequency(servo_pin,50)
servo.set_servo_pulsewidth(servo_pin, 500)

def move_servo(direction):
	duty = int((2000*direction/180)+500)
	servo.set_servo_pulsewidth(servo_pin, duty)
	time.sleep(0.1)

def move_servo90():
	move_servo(90)

def move_servo0():
	move_servo(0)



move_servo(90)
time.sleep(5)
move_servo(0)
