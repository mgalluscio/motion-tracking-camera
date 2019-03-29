import numpy as np
import cv2
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

global tiltServoAngle = 90
global panServoAngle = 100

panPin = 14
tiltPin = 15

faceFront = cv2.CascadeClassifier('../dependencies/data/haarcascade_frontalface_alt2.xml')
faceProfile = cv2.CascadeClassifier('../dependencies/data/haarcascade_profileface.xml')

cap = cv2.VideoCapture(0)

capWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
capHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

def track(servo, angle):
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dc = (angle / 20) + 3
	pwm.ChangeDutyCycle(dc)
	sleep(0.3)
	pwm.stop()

#intialize camera position

track(panPin, tiltServoAngle)
track(tiltPin, panServoAngle)

def updateServoPosition(x, y):
	global panServoAngle
	global tiltServoAngle
	if (x < ):
		panServoAngle += 10
		if panServoAngle > 140:
			panServoAngle = 140
  		track(panPin, panServoAngle)
	
	if (x > ):
		panServoAngle -= 10
		if panServoAngle < 40:
			panServoAngle = 40
        	track(panPin, panServoAngle)

	if (y < ):
		tiltServoAngle += 10
		if tiltServoAngle > 140:
			tiltServoAngle = 140
        	track(tiltPin, tiltServoAngle)
		 
	if (y > ):
        	tiltServoAngle -= 10
		if tiltServoAngle < 40:
			tiltServoAngle = 40
		track(tiltPin, tiltServoAngle)
      	

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    face = faceFront.detectMultiScale(gray, 1.3, 5)
    #profile = faceProfile.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)

    for (x,y,w,h) in face:
        print("front", x,y,w,h)
	updateServoPosition(int((x+w)/2), int((y+h)/2))
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        color = (255,0,0)
        stroke = 2
        end_front_x = x + w
        end_front_y = y + h
        cv2.rectangle(frame, (x,y), (end_front_x, end_front_y), color, stroke)
        #for (px,py,pw,ph) in profile:
            #print("profile", px,py,pw,ph)
            #roi_gray_profile = gray[py:py+h, px:px+pw]
            #roi_color_profile = frame[py:py+ph, px:px+pw]
            #color_profile = (255,0,0)
            #stroke_profile = 2
            #end_profile_x = px + pw
            #end_profile_y = py + ph
            #cv2.rectangle(frame, (px, py), (end_profile_x, end_profile_y), color_profile, stroke_profile)
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
