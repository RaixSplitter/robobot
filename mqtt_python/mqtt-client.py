#!/usr/bin/env python3
import os
import time
from enum import Enum

import numpy as np
import cv2
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

from modules.task import TaskState
from modules.axe import Axe
from modules.eight import Eight
from modules.roundabout import Roundabout

from map import master_map

class State(Enum):
	START = 0
	CAMERA_CALIBRATION = 1
	FOLLOW_LINE = 2
	LOST = 4
	TURN_LEFT = 5
	TURN_RIGHT = 6
	TRY_RECOVER = 7
	SOLVING_TASK = 8
	END_PROGRAM = 1000
	TESTING = 100000


class Task(Enum):
	AXE = 0
	EIGHT = 1
	ROUNDABOUT = 2

tasks = {
	Task.AXE : Axe(),
	Task.EIGHT : Eight(),
	Task.ROUNDABOUT : Roundabout()
}

# TODO: Move more params here
params = {
	"time_to_turn": 1.5,	# how long does a normal left or right hand turn take	
	"move_speed": 0.35		# max 1
}

# set title of process, so that it is not just called Python
setproctitle("mqtt-client")
robo_map = master_map(path = [2,6,10,3], turn ={0:None, 90:State.TURN_RIGHT, 180:State.FOLLOW_LINE, 270:State.TURN_LEFT})

def loop():
	""" """
	state = State.START
	current_task = None
	prev_state = None
	n_frames_lost = 0
	
	last_crossroad_time = time.time()
	state_start_time = time.time()

	n_images = 0		# how many images have we taken, useful for camera calibration
	max_lost_time = 1.0 # how long can we be lost before trying to recover

	# Calibration
	time_between_images = 3.0 	# seconds
	n_required_images = 40		# how many images to capture for calibration

	if not service.args.now:
		print("% Ready, press start button")
		service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
	
	# main state machine
	edge.set_line_control_targets(0, 0)
	while not (service.stop or gpio.stop()):

		if state == State.START:
			start = gpio.start() or service.args.now
			
			# start = True # NOTE: Temporary overwrite to not need to press button
			
			if start:
				state = robo_map.robot_state
				# state = State.TESTING				
				print(f"% Starting, in state: {state}")

		elif state == State.FOLLOW_LINE:
			edge.set_line_control_targets(target_velocity = move_speed, target_position = 0.0)
			# Handle losing the line and intiating recovery
			if edge.on_line:
				n_frames_lost = 0
			else:
				n_frames_lost += 1 
				
				if 5 < n_frames_lost:
					state = State.LOST
			
			# If we are at a crossroad, change node
			if edge.on_crossroad and 5.0 < time.time() - last_crossroad_time:
				robo_map.next_action(params)
				state = robo_map.robot_state
				last_crossroad_time = time.time()
		
		elif state == State.TURN_LEFT:
			edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
			service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
			if params["time_to_turn"] < time_in_state(state_start_time):
				state = State.FOLLOW_LINE

		elif state == State.TURN_RIGHT:
			edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
			service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
			if params["time_to_turn"] < time_in_state(state_start_time):
				state = State.FOLLOW_LINE

		elif state == State.LOST:
			if max_lost_time < time_in_state(state_start_time):
				state = State.TRY_RECOVER

		elif state == State.TRY_RECOVER:
			edge.set_line_control_targets(target_velocity = -0.5*move_speed, target_position = 0.0)
			if edge.on_line:
				state = State.FOLLOW_LINE
			if 10 < time_in_state(state_start_time):
				state = State.END_PROGRAM

		elif state == State.CAMERA_CALIBRATION:
			print(f"Taking image, {n_images}/{n_required_images} taken")
			time.sleep(time_between_images)
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
				print(f"Succeeded subtask '{current_task}'")
				state = State.END_PROGRAM # TEmporary

			if 100 < time_in_state(state_start_time):
				print(f"Lost in task {current_task}")
				state = State.END_PROGRAM

		elif state == State.END_PROGRAM:
			print("Ending program")
			break

		elif state == State.TESTING:
			im = take_image(save = False, show = True)
			from modules.aruco import get_pose
			save_path = "images/poses.png"
			poses = get_pose(im, save_path)
			# state = State.SOLVING_TASK
			# current_task = Task.ROUNDABOUT
			state = State.END_PROGRAM
			pass

		else:
			print(f"Unknown state '{state}', ending program")
			break

		# NOTE: You cant watch stream in vscode instance, must be in ssh -X ... forwarding stream in terminal
		# NOTE: We dont want to stream video while moving normally as it tanks the update speed
		# if os.environ.get('DISPLAY'):
		# 	stream_video(draw_debug_overlay=True)
		
		if state != prev_state:
			print(f"% Changed state from {prev_state} to {state}")
			state_start_time = time.time()
			prev_state = state

		# tell interface that we are alive
		service.send(service.topicCmd + "ti/alive", str(service.startTime))
		time.sleep(0.025) # 40 Hz


	# end of mission, turn LEDs off and stop
	service.send(service.topicCmd + "T0/leds","16 0 0 0") 
	gpio.set_value(20, 0)
	edge.set_line_control_targets(0,0) # stop following line
	service.send(service.topicCmd + "ti/rc","0 0")
	time.sleep(0.05)


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


def take_image(save: bool = True, show: bool = True) -> None | np.ndarray:
	""" """
	if not cam.useCam:
		return None

	ok, img, imgTime = cam.getImage()
	if not ok:
		if cam.imageFailCnt < 5:
			print("% Failed to get image.")
			return None
	else:
		h, w, ch = img.shape
		if not service.args.silent:
			# print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
			pass
		# TODO: Commented out
		# edge.paint(img)
		if not gpio.onPi and show:
			cv2.imshow('frame for analysis', img)
		if save:
			fn = f"images/image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
			cv2.imwrite(fn, img)
			if not service.args.silent:
				print(f"% Saved image {fn}")
		return img


def time_in_state(start_start_time: float) -> float:
	return time.time() - start_start_time


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
