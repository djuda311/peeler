import RPi.GPIO as GPIO
import time
import os
import cv2
import glob




cv_img = []
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
for img in glob.glob("/home/pi/Peeler/peeler/button.jpg"):
	n = cv2.imread(img)
	cv_img.append(n)

while True:
	input_state = GPIO.input(23)
	shutdown = GPIO.input(26)
	cv2.namedWindow('n', cv2.WND_PROP_FULLSCREEN)
	cv2.setWindowProperty('n',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
	cv2.imshow('n', n)
	key = cv2.waitKey(1) & 0xFF
	if input_state == True:
		cv2.waitKey(2)
	if key == ord("q"):
		break
		cv2.destroyAllWindows()
	if input_state == False:
		cv2.destroyAllWindows()
		print ('button pressed')
		os.system("sudo python /home/pi/Peeler/peeler/peeler_16.py")
		time.sleep(0.2)
	if shutdown == False:
		#break
		cv2.destroyAllWindows()
		print('shutting down safely')
		time.sleep(1)
		os.system("sudo shutdown -h now")
