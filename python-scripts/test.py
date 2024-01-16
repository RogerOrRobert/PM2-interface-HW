import RPi.GPIO as GPIO
import time

# Pin Definitions:
servoPin = 18  # GPIO14 (physical pin 8)

dc = 10  # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(servoPin, GPIO.OUT)  # PWM pin set as output
#servo = AngularServo(14, min_angle=90, max_angle=-90)
#servo = Servo(14)
servo = GPIO.PWM(servoPin, 50) 
servo.start(0.1)
time.sleep(2)
servo.ChangeDutyCycle(90)
time.sleep(10)
servo.ChangeDutyCycle(0)

servo.stop()
GPIO.cleanup()