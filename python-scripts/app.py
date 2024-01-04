import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt_client

# Pin Definitions:
pwmPin = 14  # GPIO14 (physical pin 8)
butPin = 2   # GPIO2 (physical pin 3)

dc = 10  # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(pwmPin, GPIO.OUT)  # PWM pin set as output
pwm = GPIO.PWM(pwmPin, 50)   # Initialize PWM on pwmPin 50Hz frequency
GPIO.setup(butPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button pin set as input w/ pull-up

clockwise_angle = 7.5
anticlockwise_angle = 2.5

mqtt_connected = False

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("Motor/move_clockwise")
        client.subscribe("Motor/move_anticlockwise")
        mqtt_connected = True
    else:
        print(f"Failed to connect, return code {rc}")
        exit(1)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"Received message on topic {msg.topic}: {payload}")
    
    if msg.topic == "Motor/move_clockwise":
        move_servo(clockwise_angle)
    elif msg.topic == "Motor/move_anticlockwise":
        move_servo(anticlockwise_angle)

def connect_mqtt():
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('localhost', 8883, 60)
    return client

def move_servo(angle):
    pwm.start(angle)
    time.sleep(1)
    pwm.stop()

def hello():
    return 'Hello, Raspberry Pi Flask App!'

if __name__ == '__main__':
	while not mqtt_connected:
		try:
			print("[MQTT] Connecting to MQTT Broker...")
			mqtt_client = connect_mqtt()
			# start client loop to get messages, non-blocking 
			if mqtt_client:
				mqtt_client.loop_forever()
		except KeyboardInterrupt:
			GPIO.cleanup()
			pwm.ChangeDutyCycle(0)
		except ConnectionRefusedError:
				print("Connection refused. Retrying in 5 seeconds...")
				time.sleep(5)
