#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

#import sys
#import threading
import time as t
from enum import Enum
#import select
import numpy as np
import cv2
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from ulog import flog


class State(Enum):
	START = 0
	CAMERA_CALIBRATION = 1
	FOLLOW_LINE = 2
	LOST = 3
	END_PROGRAM = 1000


# set title of process, so that it is not just called Python
setproctitle("mqtt-client")

############################################################

def take_image(save: bool = True) -> bool:
	if not cam.useCam:
		return False

	ok, img, imgTime = cam.getImage()
	if not ok: # size(img) == 0):
		if cam.imageFailCnt < 5:
			print("% Failed to get image.")
			return False
	else:
		h, w, ch = img.shape
		if not service.args.silent:
			# print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
			pass
		# TODO: Commented out
		# edge.paint(img)
		# cv2.imshow('frame for analysis', img)
		if not gpio.onPi:
			cv2.imshow('frame for analysis', img)
		if save:
			fn = f"images/image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
			cv2.imwrite(fn, img)
			if not service.args.silent:
				print(f"% Saved image {fn}")
		return True


def time_in_state(start_start_time: float) -> float:
	return t.time() - start_start_time


def loop():
	""" """
	state = State.START
	prev_state = None

	state_start_time = t.time()
	n_images = 0
	n_frames_lost = 0

	if not service.args.now:
		print("% Ready, press start button")
		service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
	
	# main state machine
	edge.set_line_control_targets(0, 0) # make sure line control is off
	while not (service.stop or gpio.stop()):

		pose.printPose()

		if state == State.START:
			start = gpio.start() or service.args.now
			# start = True
			if start:
				print("% Starting")
				state = State.FOLLOW_LINE
				# service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
				# service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turnrate rad/sec)
				# # follow line (at 0.25cm/s)
				# edge.lineControl(0.25, 0.0) # m/s and position on line -2.0..2.0
				# # pose.tripBreset() # use trip counter/timer B
				# pose.printPose()


		elif state == State.FOLLOW_LINE:
			edge.set_line_control_targets(target_velocity = 0.1, target_position = 0.0)
			if not edge.lineValid:
				state = State.LOST
        
		elif state == State.LOST:
			print(f"Robot lost")
			edge.set_line_control_targets(target_velocity = -0.1, target_position = 0.0)
			if edge.lineValid:
				state = State.FOLLOW_LINE

		elif state == State.CAMERA_CALIBRATION:
			print(f"Taking image, {n_images} taken")
			t.sleep(3.0)
			take_image(save = True)

			n_images += 1
			if n_images == 20:
				state = State.END_PROGRAM
			
			# Check if something has gone wrong
			if not cam.useCam or 30 < time_in_state(state_start_time):
				state = State.END_PROGRAM

		# elif state == 12: # following line
		# 	if edge.lineValidCnt == 0: #  or pose.tripBtimePassed() > 10:
		# 		n_frames_lost += 1
		# 		if 10 < n_frames_lost:
		# 			print("ACTUALLY LOST")
		# 			state = 20
		# 		# no more line
		# 		# edge.lineControl(0,0) # stop following line
		# 		# pose.tripBreset()
		# 		# service.send(service.topicCmd + "ti/rc","0.1 0.5") # turn left
		# 		print("LOST LINE", pose.tripBtimePassed(), edge.lineValidCnt)
		# 		# state = 12 # turn left
		# 	else:
		# 		n_frames_lost = 0

		# elif state == 14: # turning left
		# 	if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 10:
		# 		state = 20 # finished   =17 go look for line
		# 		service.send(service.topicCmd + "ti/rc","0 0") # stop for images
		# 	print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")

		# allow openCV to handle imshow (if in use)
		# images are almost useless while turning, but
		# used here to illustrate some image processing (painting)
		if cam.useCam:
			take_image(save = False)
			key = cv2.waitKey(100) # ms
			if key > 0: # e.g. Esc (key=27) pressed with focus on image
				break
		
		if state != prev_state:
			print(f"% Changed state from {prev_state} to {state}")
			state_start_time = t.time()
			prev_state = state

		# note state change and reset state timer
		# if state.value != oldstate:
		# 	flog.write(state.value)
		# 	flog.writeRemark(f"% State change from {oldstate} to {state}")
		# 	print(f"% State change from {oldstate} to {state}")
		# 	oldstate = state
		# 	stateTime = datetime.now()
		# do not loop too fast
		t.sleep(0.1)
		# tell interface that we are alive
		service.send(service.topicCmd + "ti/alive", str(service.startTime))

	# end of mission, turn LEDs off and stop
	service.send(service.topicCmd + "T0/leds","16 0 0 0") 
	gpio.set_value(20, 0)
	edge.set_line_control_targets(0,0) # stop following line
	service.send(service.topicCmd + "ti/rc","0 0")
	t.sleep(0.05)

############################################################

if __name__ == "__main__":
	print("% Starting")
	# where is the MQTT data server:
	service.setup('localhost') # localhost
	#service.setup('10.197.217.81') # Juniper
	#service.setup('10.197.217.80') # Newton
	#service.setup('10.197.218.172')
	if service.connected:
		loop()
		service.terminate()
	print("% Main Terminated")
