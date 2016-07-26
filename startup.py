import RPi.GPIO as GPIO
import time
import os
import cv2

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

cv2.namedWindow('img', cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('img',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
img = cv2.imread("button.jpg")
cv2.imshow('img', img)


while True:
	input_state = GPIO.input(23)
	cv2.namedWindow('img', cv2.WND_PROP_FULLSCREEN)
	cv2.setWindowProperty('img',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
	cv2.imshow('img', img)
	if input_state == True:
		cv2.waitKey(5)
	if input_state == False:
		cv2.destroyAllWindows()
		print ('button pressed')
		os.system("sudo python /home/pi/Peeler/peeler/peeler_16.py")
		time.sleep(0.2)