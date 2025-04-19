from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from modules.ball_detection import *
from modules.aruco import get_pose
from scam import cam
import time
import logging
from enum import Enum
from dataclasses import dataclass
import cv2

logging.basicConfig(
	filename="navigate_to_pose_log",
	filemode="w",
	format="%(asctime)s,%(msecs)03d %(name)s %(levelname)s %(message)s",
	datefmt="%Y-%m-%d %H:%M:%S",
	level=logging.DEBUG,
)
LOGGER = logging.getLogger(__name__)

class State(Enum):
	INIT = 'INITIALIZE SEQUENCE'
	LOWER_ARM = 'LOWER ARM'
	WAIT = 'WAIT FOR CAR'
	INTOPOS = 'INTO POSITION'
	ROTATION = 'Rotation'
	JANK_PREPARE = 'JANK PREP'
	JANK = 'JANKING'


class RetrieveLuggage(Task):
	def __init__(self):
		super().__init__(name="Navigate")
		#region CONFIG
		self.SPEED = 0.1
		self.TURNRATE = 0.3
  
		self.DEFAULT_STATE = State.WAIT
		self.DIST_TO_WALL = 0.12 #[M]
		self.ROTATION = np.pi * 1/4
		self.ORANGE_MIN = np.array([3,184,120])
		self.ORANGE_MAX = np.array([29,255,255])
		self.ORANGE_THRESHHOLD = 50
		self.last_car_pose = 0
		#endregion
  
		#region States
		self.states_q : list[State] = []
		self.state: State = State.INIT
		self.finish = False
		self.actions = {
			State.INIT 			: self.initialize,
			State.INTOPOS 		: self.get_into_position,
			State.LOWER_ARM 	: self.lower_arm,
			State.WAIT 			: self.wait,
			State.ROTATION 		: self.rotate,
			State.JANK_PREPARE 	: self.jank_prepare,
			State.JANK 			: self.jank,
   

		}
		#endregion

	#region State Logic
	def change_state(self) -> None:
		if self.states_q: #Get next state
			self.state = self.states_q.pop(0)
		else: #Default state
			self.state = self.default_state
		LOGGER.debug(f"State: {self.state}, State Queue: {self.states_q}")

	def add_state(self, state) -> None:
		self.states_q.append(state)

	#endregion

	#region Default State Behavior
	def stop(self):
		service.send(service.topicCmd + "ti/rc", "0.0 0.0")
  
	def drive(self, reverse : bool = False):
		"""
		Forward or backwards driving

		ARGS:
			reverse : bool, if False fowards driving, if True reverse driving.
		RETURN:
			None
  		"""
		if reverse: #Backwards
			service.send(service.topicCmd + "ti/rc", f"{-self.SPEED} 0.0")
		else: #Forward
			service.send(service.topicCmd + "ti/rc", f"{self.SPEED} 0.0")
	def turn(self, left : bool = False):
		"""
		Turns the robot either left or right

		ARGS:
			left : bool, if False turns left, if True turns right.
		RETURN:
			None
  		"""
		if left: #Turns left
			service.send(service.topicCmd + "ti/rc", f"0.0 {self.TURNRATE}")
	  
	  
		else: # Turns right
			service.send(service.topicCmd + "ti/rc", f"0.0 {-self.TURNRATE}")


	def loop(self, detection_target: str = "ball"):
		action = self.actions[self.state]
		error = action()

		if self.finish:
			return TaskState.SUCCESS

		if error:
			LOGGER.error(f"Failed to execute state action: {self.state} for action {action}")
			return TaskState.FAILURE
		return TaskState.EXECUTING
	#endregion
 
	def initialize(self):
		# ok, img, _ = cam.getImage()
		# if ok:
		# 	poses = get_pose(img, "captured_image.jpg")
		# 	cv2.imwrite("captured_image.jpg", img)
		# 	if poses:
		# 		# Check if the car is in the image
		# 		rvec, tvec, identifier = poses.get("car", (None, None, None))
    
		# 		if identifier:
		# 			if tvec[0][0] > self.last_car_pos:
		# 				self.stop()
		# 				self.add_state(State.INTOPOS)
		# 				self.change_state()
		# 			self.last_car_pos = tvec[0][0]
	 
		# 	#If the car is not in the image, we can assume that the car is not in the image
		# 	elif self.last_car_pos :
				
		# 		if total_difference < 0:
					
		# 			return
		self.stop()
		self.add_state(State.INTOPOS)
		self.change_state()
		return
		

	def get_into_position(self):
		distance = ir.ir[1] #IR distance to object in front
  
		print(distance, self.DIST_TO_WALL)
		
		if distance > self.DIST_TO_WALL:
			self.drive()
			return

		pose.tripBreset()

		self.stop()
		self.add_state(State.LOWER_ARM)
		self.change_state()
		return

	def rotate(self):
		if abs(pose.tripBh) >= abs(self.ROTATION):
			self.stop()
			self.add_state(State.WAIT)
			self.change_state()
			return
		else:
			self.turn(left = False)
			return

	def lower_arm(self):
		service.send(service.topicCmd + "T0/servo", "1 -650 200")
		self.add_state(State.JANK_PREPARE)
		self.change_state()
		return

	def jank_prepare(self):
		print("Din mor er en luder")
		ok, img, imgstate = cam.getImage()
  
		# Save the captured image for debugging or analysis
		cv2.imwrite("captured_image.jpg", img)
  
		if ok:
			# poses = get_pose(img, "captured_image.jpg")
			# print(poses,ir.ir[1])

			#Take average of vertical 
			# Convert the image to HSV color space
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
			# Threshold the image to get only orange colors
			mask = cv2.inRange(hsv, self.ORANGE_MIN, self.ORANGE_MAX)
   
			# Save the mask for debugging or analysis
			cv2.imwrite("orange_mask.jpg", mask)
			print("gulerød", np.min(mask), np.max(mask), mask.dtype, mask.shape)
	
			
			mean_v_mask = np.mean(mask, axis = 1)

			# Save the vertical mean mask for debugging or analysis
			np.savetxt("vertical_mean_mask.txt", mean_v_mask, fmt="%.2f")
   
			left_window = mean_v_mask[:int(len(mean_v_mask)/5)]
			right_window = mean_v_mask[int(len(mean_v_mask)/5):]
   
			condition1 = np.mean(left_window) > self.ORANGE_THRESHHOLD
			condition2 = np.mean(right_window) > self.ORANGE_THRESHHOLD
			print(condition1, condition2)
			print(np.mean(left_window), np.mean(right_window))
			if condition1 and condition2:
				self.stop()
				self.add_state(State.JANK)
				self.change_state()
				pose.tripBreset()
				return
			

			# if car_pose:
			# 	rvec, tvec, identifier = car_pose
			# 	print("SWAPPING STATES")
			# 	self.add_state(State.JANK)
			# 	self.change_state()
			# 	return

	def jank(self):
		if abs(pose.tripB) >= 0.3:
			self.stop()
			# self.finish = True
			return
		else:
			self.drive(reverse=True)
	 
			
			
			
		

	def wait(self):
		ok, img, imgstate = cam.getImage()
		if ok:
			pose = get_pose(img, "captured_image.jpg")
		self.stop()


