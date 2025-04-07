#!/usr/bin/env python3
import os
import time
from enum import Enum

import cv2
import numpy as np
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
from modules.ball_detection import pose_est_ball_from_img

from map import master_map,N,E,S,W

class State(Enum):
	START = 0
	PHOTO_MODE = 1
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
	"time_to_turn": 2, # how long does a normal left or right hand turn take	
	"move_speed": 0.25, # max 1
}
all_poses = []
# speed        | 0.1 | 0.2 | 0.3
# 180 deg turn | 4.5 | 4.0 | 4.2

# set title of process, so that it is not just called Python
setproctitle("mqtt-client")
robo_map = master_map(
	path = [2,6,10,3], 
	turn = {
		0:None, 90:State.TURN_RIGHT, 180:State.FOLLOW_LINE, 270:State.TURN_LEFT
	}, 
	robot_param=params
)


def loop():
	""" """
	# state = State.START
	state = State.TESTING
	current_task = None
	prev_state = None
	n_frames = 0
	n_frames_lost = 0

	last_crossroad_time = time.time()
	state_start_time = time.time()
	start_time = time.time() # for fps calculation

	n_images = 0		# how many images have we taken, useful for PHOTO MODE
	max_lost_time = 1.0 # how long can we be lost before trying to recover

	# Calibration
	time_between_images = 3.0 	# seconds
	n_required_images = 3		# how many images to capture for calibration

	if not service.args.now:
		print("% Ready, press start button")
		service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
	
	# main state machine
	edge.set_line_control_targets(0, 0)
	while not (service.stop or gpio.stop()):
		n_frames += 1

		if state == State.START:
			start = gpio.start() or service.args.now
			# start = True # NOTE: Temporary overwrite to not need to press button
			
			if start:
				state = robo_map.robot_state
				print(f"% Starting, in state: {state}")

		elif state == State.FOLLOW_LINE:
			edge.set_line_control_targets(target_velocity = params["move_speed"], target_position = 0.0)
			# Handle losing the line and intiating recovery
			if edge.on_line:
				n_frames_lost = 0
			else:
				n_frames_lost += 1 
				
				if 5 < n_frames_lost:
					state = State.LOST
			
			# If we are at a crossroad, change node
			if edge.on_crossroad and 5.0 < time.time() - last_crossroad_time:
				robo_map.next_action()
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
			edge.set_line_control_targets(target_velocity = -0.5*params["move_speed"], target_position = 0.0)
			if edge.on_line:
				state = State.FOLLOW_LINE
			if 10 < time_in_state(state_start_time):
				state = State.END_PROGRAM

		elif state == State.PHOTO_MODE:
			print(f"Taking image, {n_images}/{n_required_images} taken")
			# time.sleep(time_between_images)
			input() # enter in terminal to proceed
			take_image(save = True)

			n_images += 1
			if n_images == n_required_images:
				state = State.END_PROGRAM
			
			# Check if something has gone wrong, if we should 
			# if not cam.useCam or 1.5 * (time_between_images * n_images) < time_in_state(state_start_time):
			# 	state = State.END_PROGRAM

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

		# NOTE: This state is the catch all for any misc testing code
		elif state == State.TESTING:
			# edge.set_line_control_targets(target_velocity = params["move_speed"], target_position = 0.0)
			pass

		else:
			print(f"Unknown state '{state}', ending program")
			break

		# NOTE: You cant watch stream in vscode instance, must be in ssh -X ... forwarding stream in terminal
		# NOTE: We dont want to stream video while moving normally as it tanks the update speed
		# NOTE: Plotting anything drops the FPS making the dynamics of for instance the PID controllers act differently
		if os.environ.get('DISPLAY'):
			# Plot map of where we think we are
			# all_poses.append(pose.pose.copy())
			# img = draw_trajectory(all_poses)
			# cv2.imshow("Trajectory", img)
			# cv2.waitKey(1)

			# Stream camera feed. NOTE: Tanks the rest of the program
			# stream_video(draw_debug_overlay=True)
			pass

		if state != prev_state:
			print(f"% Changed state from {prev_state} to {state}")
			state_start_time = time.time()
			prev_state = state

		# tell interface that we are alive
		service.send(service.topicCmd + "ti/alive", str(service.startTime))
		time.sleep(0.025) # 40 Hz

		# Print FPS
		if n_frames % 10 == 0:
			elapsed_time = time.time() - start_time
			fps = 10 / elapsed_time
			print(f"FPS: {fps:.2f}")
			start_time = time.time()  # Reset timer


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
	img = cv2.flip(img, 0)
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
	img = cv2.flip(img, 0)

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


def draw_trajectory(poses, img_size=(100, 100), border=20):
	"""
	Draws a visualization of the 2D pose path using OpenCV.
	
	:param poses: List of (x, y, heading) tuples.
	:param img_size: Size of the output image (width, height).
	:param border: Border padding to avoid points touching the edges.
	:return: OpenCV image with the drawn path.
	"""
	if not poses:
		return np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8)

	# Extract x, y coordinates
	x_vals = [p[0] for p in poses]
	y_vals = [p[1] for p in poses]

	# Find min/max for scaling
	min_x, max_x = min(x_vals), max(x_vals)
	min_y, max_y = min(y_vals), max(y_vals)

	# Compute scale factors to fit within the image
	scale_x = (img_size[0] - 2 * border) / (max_x - min_x + 1e-6)
	scale_y = (img_size[1] - 2 * border) / (max_y - min_y + 1e-6)
	scale = min(scale_x, scale_y)  # Keep aspect ratio

	# Convert to pixel coordinates (flip y-axis since OpenCV has origin at top-left)
	def to_pixel(x, y):
		px = int((x - min_x) * scale + border)
		py = int(img_size[1] - ((y - min_y) * scale + border))  # Invert Y for OpenCV
		return px, py

	# Create a blank image
	img = np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8)

	# Draw path
	for i in range(1, len(poses)):
		p1 = to_pixel(*poses[i - 1][:2])
		p2 = to_pixel(*poses[i][:2])
		cv2.line(img, p1, p2, (0, 255, 255), 2)  # Yellow path

	# Draw current position and heading
	x, y, heading, _ = poses[-1]
	px, py = to_pixel(x, y)
	cv2.circle(img, (px, py), 5, (255, 0, 0), -1)  # Blue dot for current position

	# Draw heading arrow
	arrow_length = 20  # Adjust arrow length
	dx = int(arrow_length * np.cos(heading))
	dy = int(arrow_length * np.sin(heading))
	cv2.arrowedLine(img, (px, py), (px + dx, py - dy), (0, 255, 0), 2)  # Green arrow

	return img


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
