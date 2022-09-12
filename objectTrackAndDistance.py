#packages
from __future__ import print_function
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import os
import RPi.GPIO as GPIO

# Servos GPIOs
panServo = 27
tiltServo = 17

# initialize LED GPIO
led = 2
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)

#initialize buzzer
buzzer = 21
GPIO.setup(buzzer, GPIO.OUT)

#initialize sonar
PIN_TRIGGER = 4
PIN_ECHO = 22

GPIO.setup(PIN_TRIGGER, GPIO.OUT)
GPIO.setup(PIN_ECHO, GPIO.IN)

GPIO.output(PIN_TRIGGER, GPIO.LOW)


#measure distance
def measure():
    print ("Waiting for sensor to settle")

    time.sleep(2)

    print ("Calculating distance")

    GPIO.output(PIN_TRIGGER, GPIO.HIGH)

    time.sleep(0.00001)

    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    while GPIO.input(PIN_ECHO)==0:
        pulse_start_time = time.time()
    while GPIO.input(PIN_ECHO)==1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    print ("Object detected and its Distance is:",distance,"cm")

    
#position servos 
def positionServo (servo, angle):
    os.system("python angleServoCtrl.py " + str(servo) + " " + str(angle))
    print("[log] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))

# position servos to present object at center of the frame
def mapServoPosition (x, y):
    global panAngle
    global tiltAngle
    if (x < 220):
        panAngle += 10
        if panAngle > 140:
            panAngle = 140
        positionServo (panServo, panAngle)
 
    if (x > 280):
        panAngle -= 10
        if panAngle < 40:
            panAngle = 40
        positionServo (panServo, panAngle)

    if (y < 160):
        tiltAngle += 10
        if tiltAngle > 140:
            tiltAngle = 140
        positionServo (tiltServo, tiltAngle)
 
    if (y > 210):
        tiltAngle -= 10
        if tiltAngle < 40:
            tiltAngle = 40
        positionServo (tiltServo, tiltAngle)

# initialize the video stream and allow the camera sensor to warmup
print("[log] waiting for camera to warmup...")
vs = VideoStream(0).start()
time.sleep(2.0)

#red
colorLower = (169, 100, 100)
colorUpper = (189, 255, 255)

#green
#colorLower = (24, 100, 100)
#colorUpper = (44, 255, 255)

# Start with LED off
GPIO.output(led, GPIO.LOW)
ledOn = False

# Initialize angle servos at 90-90 position
global panAngle
panAngle = 90
global tiltAngle
tiltAngle =90

# positioning Pan/Tilt servos at initial position
positionServo (panServo, panAngle)
positionServo (tiltServo, tiltAngle)

# loop over the frames from the video stream
while True:
	# frame, and convert it to the HSV color space
	frame = vs.read()
	frame = imutils.resize(frame, width=500)
	frame = imutils.rotate(frame, angle=180)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask 
	mask = cv2.inRange(hsv, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the object
	# contour is an area of consecutive lines
	# cv2.findContours(src, retrieval mode, approx method)
	# it returns a numpy array
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			measure()
			
			# position Servo at center of circle
			mapServoPosition(int(x), int(y))
			
			# if the led is not already on, turn the LED on
			if not ledOn:
				GPIO.output(led, GPIO.HIGH)
				ledOn = True
				#buzzer start
				p = GPIO.PWM(buzzer, 50)  # channel=21 frequency=50Hz
				p.start(0)
				p.ChangeDutyCycle(40)

	# if the object is not detected, turn the LED off
	elif ledOn:
		GPIO.output(led, GPIO.LOW)
		ledOn = False
		p.stop()

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	
	# if [ESC] key, stop the loop
	key = cv2.waitKey(1) & 0xFF
	if key == 27:
            break

# do a bit of cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
positionServo (panServo, 90)
positionServo (tiltServo, 90)
GPIO.cleanup()
cv2.destroyAllWindows()
vs.stop()
