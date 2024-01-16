import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt_client

# Pin Definitions:
servoPin = 14  # GPIO14 (physical pin 8)
redLedPin = 17   # GPIO2 (physical pin 11)
greenLedPin = 22   # GPIO2 (physical pin 15)
yellowLedPin = 27   # GPIO2 (physical pin 13)
dc = 10  # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(redLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(greenLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(yellowLedPin, GPIO.OUT)  # PWM pin set as output

# Set up PWM for each LED
red_led_pwm = GPIO.PWM(redLedPin, 100)  # 100 Hz frequency
green_led_pwm = GPIO.PWM(greenLedPin, 100)
yellow_led_pwm = GPIO.PWM(yellowLedPin, 100)

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
    if msg.topic == "Motor/move_clockwise" or msg.topic == "Motor/move_anticlockwise":
        recieved_value = float(payload)
        start_led(recieved_value)

def connect_mqtt():
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('192.168.0.38', 1883, 60)
    return client

def start_led(value):
    red_led_pwm.start(value)
    yellow_led_pwm.start(value) 
    green_led_pwm.start(value) 


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
		except ConnectionRefusedError:
				print("Connection refused. Retrying in 5 seeconds...")
				time.sleep(5)
