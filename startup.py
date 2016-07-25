import RPi.GPIO as GPIO
import time
import os

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
	input_state = GPIO.input(23)
	if input_state == False:
		print ('button pressed')
		os.system("sudo python peeler_16.py")
		time.sleep(0.2)