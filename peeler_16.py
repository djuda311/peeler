import cv2
import numpy as np
import math 
import csv
import copy
import RPi.GPIO as GPIO
cv2.destroyAllWindows()

cv2.namedWindow('Press space to capture image', cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('Press space to capture image',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)



camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size =(640,480))
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	video = frame.array
	cv2.rectangle(video,(100,300),(300,400),(0,255,0),3)
	cv2.rectangle(video,(400,300),(600,400),(0,255,0),3)
	cv2.imshow("Press space to capture image", video)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	input_state = GPIO.input(18)
	if key == ord(" "):
		break
		cv2.destroyAllWindows()
	if key == ord("q"):
		break
		cv2.destroyAllWindows()
	if input_state == False:
		break
		cv2.destroyAllWindows()
		
camera.capture(rawCapture, format="bgr")
img = rawCapture.array
cv2.setWindowProperty("image",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow("image", img)
cv2.imwrite('picam1.png',img)
picam = cv2.imread('picam1.png')
#img = cv2.imread('picam1.PNG')
#cv2.waitKey()
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#cv2.waitKey()
imgl = copy.copy(img_gray)
imgr = copy.copy(imgl)
img_left = imgl[300:400, 100:300]
#cv2.waitKey()
img_right = imgr[300:400, 400:600]
cv2.namedWindow("img_left", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("img_left",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow("img_left", img_left)
cv2.namedWindow("img_right", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("img_right",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow("img_right", img_right)
#cv2.waitKey()

retl, threshl = cv2.threshold(img_left,170,255,cv2.THRESH_BINARY) #converts image to black and white based on a min brightness specified
retr, threshr = cv2.threshold(img_right,170,255,cv2.THRESH_BINARY) #converts image to black and white based on a min brightness specified
threshl = cv2.medianBlur(threshl, 1)
threshr = cv2.medianBlur(threshr, 1)
cv2.imshow('threshr',threshr) #shows the black and white image
cv2.imshow('threshl',threshl) #shows the black and white image
cv2.waitKey()

# ---- The contours function finds the line and turns it into a contour of binary points, 0's and 1's. ------
# Because the line isn't consistent and is broken in a few places, the function finds multiple contours. 
# im2 is the new image with the contours, contours is a list of each of the contours where contours[0] is a 2d array of each point in the matrix like on a plot, hierarchy is a list of all of the contours found

im2l, contoursl, hierarchyl = cv2.findContours(threshl,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
im2r, contoursr, hierarchyr = cv2.findContours(threshr,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
#print contoursr[0]

cntl = contoursl[0] #the contours function finds a lot of contours because the line isn't consistent. contours[0] is the first.
cntr = contoursr[0]
xl, yl, zl = hierarchyl.shape #y-1 is the number of contours found
xr, yr, zr = hierarchyr.shape #y-1 is the number of contours found

bl = yl-1 # There is an extra row in the heirarchy array so the total number of contours is 1 less than the y size of the matrix. 
br = yr-1

# ---- Add all of the contours together into one longer array. "concatenate" means to add to the end of the array. 
if yl == 1:
	cntl = contoursl[0]
else:
	for i in range(bl):
		cntl = np.concatenate((cntl,contoursl[i+1]), axis=0)
if yr == 1:
	cntr = contoursr[0]
else:
	for i in range(br):
		cntr = np.concatenate((cntr,contoursr[i+1]), axis=0)



rows_l,cols_l = img_left.shape[:2]
[vxl,vyl,xl,yl] = cv2.fitLine(cntl, cv2.DIST_L2,0,0.01,0.01)
leftyl = int((-xl*vyl/vxl) + yl)
rightyl = int(((cols_l-xl)*vyl/vxl)+yl)
imglline = cv2.line(img_left,(cols_l-1,rightyl),(0,leftyl),(255,250,0),1)
cv2.namedWindow("imglline", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("imglline",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow('imglline', imglline)
cv2.waitKey()

rows_r,cols_r = imgr.shape[:2]
[vxr,vyr,xr,yr] = cv2.fitLine(cntr, cv2.DIST_L2,0,0.01,0.01)
leftyr = int((-xr*vyr/vxr) + yr)
rightyr = int(((cols_r-xr)*vyr/vxr)+yr)
imgrline = cv2.line(img_right,(cols_r-1,rightyr),(0,leftyr),(255,250,0),1)
cv2.namedWindow("imgrline", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("imgrline",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow('imgrline', imgrline)
cv2.waitKey()

slopel = -vyl/vxl
slopel = math.degrees(slopel)
sloper = -vyr/vxr
sloper = math.degrees(sloper)

cv2.putText(imgl, "Slope = %s deg" % slopel, (4,13), cv2.FONT_HERSHEY_SIMPLEX, .3,(255,255,255),1, cv2.LINE_AA)
epsilonl = 0.1*cv2.arcLength(cntl,True)
approxl = cv2.approxPolyDP(cntl,epsilonl,False)

cv2.putText(imgr, "Slope = %s deg" % sloper, (4,13), cv2.FONT_HERSHEY_SIMPLEX, .3,(255,255,255),1, cv2.LINE_AA)
epsilonr = 0.1*cv2.arcLength(cntr,True)
approxr = cv2.approxPolyDP(cntr,epsilonr,False)
cv2.destroyAllWindows()
angle_between_blades = 180 - slopel + sloper
cv2.putText(img, "Angle Between Blades = %s deg" % angle_between_blades, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1, cv2.LINE_AA)
x,y,w,h = cv2.boundingRect(cntl)
cv2.rectangle(imgl,(x,y),(x+w,y+h),(255,255,255),2)
lengthl = ((w**2)+(h**2))**(1/2)
cv2.putText(img, "Left Blade Length = %s pixels" % w, (30,60), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1, cv2.LINE_AA)
x,y,w,h = cv2.boundingRect(cntr)
cv2.rectangle(imgr,(x,y),(x+w,y+h),(255,255,255),2)
lengthr = ((w**2)+(h**2))**(1/2)
cv2.putText(img, "Right Blade Length = %s pixels" % w, (30,90), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1, cv2.LINE_AA)
cv2.namedWindow('img', cv2.WND_PROP_FULLSCREEN)

cv2.setWindowProperty('img',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

cv2.imshow('img', img)
cv2.waitKey()
