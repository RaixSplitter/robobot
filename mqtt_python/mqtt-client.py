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

from mqtt_python.modules.task import TaskState
from mqtt_python.modules.axe import Axe

class State(Enum):
	START = 0
	CAMERA_CALIBRATION = 1
	FOLLOW_LINE = 2
	LOST = 3
	TURN_LEFT = 4
	TURN_RIGHT = 5
	TRY_RECOVER = 6
	SOLVING_TASK = 7
	END_PROGRAM = 1000


class Task(Enum):
	AXE = 0

tasks = {
	Task.AXE : Axe()
}

# set title of process, so that it is not just called Python
setproctitle("mqtt-client")

def loop():
	""" """
	state = State.START
	current_task = None
	prev_state = None

	state_start_time = t.time()
	n_images = 0		# how many images have we taken, useful for camera calibration
	time_to_turn = 0.5	# how long does a normal left or right hand turn take
	max_lost_time = 1.0 # how long can we be lost before trying to recover
	move_speed = 0.1

	# Calibration
	time_between_images = 3.0 	# seconds
	n_required_images = 40		# how many images to capture for calibration


	if not service.args.now:
		print("% Ready, press start button")
		service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
	
	# main state machine
	edge.set_line_control_targets(0, 0)
	while not (service.stop or gpio.stop()):

		# pose.printPose()

		if state == State.START:
			start = gpio.start() or service.args.now
			
			# start = True # NOTE: Temporary overwrite to not need to press button
		
			if start:
				state = State.CAMERA_CALIBRATION
				print(f"% Starting, in state: {state}")
				# state = State.FOLLOW_LINE

		elif state == State.FOLLOW_LINE:
			edge.set_line_control_targets(target_velocity = move_speed, target_position = 0.0)
			if not edge.on_line:
				state = State.LOST
			
			# TODO: Here we want some logic to determine whether to turn left or right
			if edge.on_crossroad:
				state = State.TURN_LEFT
        
		elif state == State.TURN_LEFT:
			edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
			service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
			if time_to_turn < time_in_state(state_start_time):
				state = State.FOLLOW_LINE

		elif state == State.TURN_RIGHT:
			edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
			service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
			if time_to_turn < time_in_state(state_start_time):
				state = State.FOLLOW_LINE

		elif state == State.LOST:
			if max_lost_time < time_in_state(state_start_time):
				state = State.TRY_RECOVER

		elif state == State.TRY_RECOVER:
			edge.set_line_control_targets(target_velocity = -0.1, target_position = 0.0)
			if edge.on_line:
				state = State.FOLLOW_LINE

		elif state == State.CAMERA_CALIBRATION:
			print(f"Taking image, {n_images}/{n_required_images} taken")
			t.sleep(time_between_images)
			take_image(save = True)

			n_images += 1
			if n_images == n_required_images:
				state = State.END_PROGRAM
			
			# Check if something has gone wrong, if we should 
			if not cam.useCam or 1.5 * (time_between_images * n_images) < time_in_state(state_start_time):
				state = State.END_PROGRAM

		elif state == State.SOLVING_TASK:
			if current_task is None:
				print("Shouldnt happen you fucked up")
				state = State.END_PROGRAM
				continue

			sub_state = tasks[current_task].loop()
			if sub_state == TaskState.FAILURE:
				pass
			elif sub_state == TaskState.EXECUTING:
				pass
			elif sub_state == TaskState.LOST:
				pass
			elif sub_state == TaskState.SUCCESS:
				pass

			if 100 < time_in_state(state_start_time):
				print(f"Lost in task {current_task}")
				state = State.END_PROGRAM


		elif state == State.END_PROGRAM:
			print("Ending program")
			break

		else:
			print(f"Unknown state '{state}', ending program")
			break

		# NOTE: This fails in vscode instance, must be in ssh -X ... forwarding stream
		# stream_video()
		
		if state != prev_state:
			print(f"% Changed state from {prev_state} to {state}")
			state_start_time = t.time()
			prev_state = state

		# tell interface that we are alive
		service.send(service.topicCmd + "ti/alive", str(service.startTime))
		t.sleep(0.1)


	# end of mission, turn LEDs off and stop
	service.send(service.topicCmd + "T0/leds","16 0 0 0") 
	gpio.set_value(20, 0)
	edge.set_line_control_targets(0,0) # stop following line
	service.send(service.topicCmd + "ti/rc","0 0")
	t.sleep(0.05)


def stream_video(draw_debug_overlay: bool = False) -> bool: 
	"""
	Stream video to X11. Remember to run ssh with x-port forwarding
		ssh -X local@10.197.218.176
	"""
	if not cam.useCam:
		return False
	ok, img, imgTime = cam.getImage()
	if not ok:
		return False
	if draw_debug_overlay:
		edge.paint(img)
	cv2.imshow('Video stream', img)
	cv2.waitKey(1)
	return True


def take_image(save: bool = True) -> bool:
	""" """
	if not cam.useCam:
		return False

	ok, img, imgTime = cam.getImage()
	if not ok:
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
