import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt_client
import threading

# Pin Definitions:
servoPin = 14  # GPIO14 (physical pin 8)
redLedPin = 17   # GPIO2 (physical pin 11)
greenLedPin = 22   # GPIO2 (physical pin 15)
yellowLedPin = 27   # GPIO2 (physical pin 13)

# Pin Setup:
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(redLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(greenLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(yellowLedPin, GPIO.OUT)  # PWM pin set as output
GPIO.setup(servoPin, GPIO.OUT)  # PWM pin set as output

# Set up PWM for each LED
red_led_pwm = GPIO.PWM(redLedPin, 100)  # 100 Hz frequency
green_led_pwm = GPIO.PWM(greenLedPin, 100)
yellow_led_pwm = GPIO.PWM(yellowLedPin, 100)
servo = GPIO.PWM(servoPin, 50)   # Initialize PWM on pwmPin 50Hz frequency

servo_value = 0
mqtt_connected = False
servo.start(0)

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        print("Connected to MQTT Broker!")
        #client.subscribe("Motor/move_clockwise")
        client.subscribe("motor")
        mqtt_connected = True
    else:
        print(f"Failed to connect, return code {rc}")
        exit(1)

def on_message(client, userdata, msg):    
    payload = msg.payload.decode()
    print(f"Received message on topic {msg.topic}: {payload}")   
    if msg:
        recieved_value = float(payload)
        print("recieved value: ", recieved_value)
        #move_servo(recieved_value)
        servo_leds(recieved_value)

def connect_mqtt():
    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('192.168.0.28', 1883, 60) #'192.168.86.45'
    return client

def servo_leds(value):
    global servo_value

    def move_servo_thread(pos):
        ms = ((2/180)*pos) + 0.5
        dc = (ms/20)*100
        servo.ChangeDutyCycle(dc)
        time.sleep(4)

    def activate_leds_thread(led_order):
        for led_pin in led_order:
            GPIO.output(led_pin, 1)
            time.sleep(1)
            GPIO.output(led_pin, 0)

    if value > servo_value:
        led_order = [redLedPin, yellowLedPin, greenLedPin]
        print("heading to 180")
    elif value <= servo_value:
        led_order = [greenLedPin, yellowLedPin, redLedPin]
        print("heading to 0")

    # Create threads for moving servo and activating LEDs
    servo_thread = threading.Thread(target=move_servo_thread, args=(value,))
    leds_thread = threading.Thread(target=activate_leds_thread, args=(led_order,))

    # Start both threads
    leds_thread.start()
    servo_thread.start()

    # Wait for both threads to finish
    leds_thread.join()
    servo_thread.join()

    servo_value = value


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
