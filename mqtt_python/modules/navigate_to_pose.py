from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from modules.ball_detection import *
from modules.aruco import drop_point, get_pose
from scam import cam
import time
import logging
from enum import Enum
from dataclasses import dataclass

logging.basicConfig(
	filename="navigate_to_pose_log",
	filemode="w",
	format="%(asctime)s,%(msecs)03d %(name)s %(levelname)s %(message)s",
	datefmt="%Y-%m-%d %H:%M:%S",
	level=logging.DEBUG,
)
LOGGER = logging.getLogger(__name__)


MAX_TRAVEL_DISTANCE = 0.4

class State(Enum):
	LOOKING_FOR_OBJECT = 'LOOKING FOR OBJECT'
	TARGET_TURN = 'TARGET TURN'
	TURN_LEFT = 'TURN LEFT'
	TURN_RIGHT = 'TURN RIGHT'
	FORWARD = 'FORWARD'
	REVERSE = 'REVERSE'
	CAPTURE = 'CAPTURE'
class PoseTarget(Enum):
	ARUCO = 'aruco'
	ARUCO_LA = 'LA'
	ARUCO_LD = 'LD'
	BLUE_BALL = 'blue'
	ORANGE_BALL = 'orange'
	A = 'A'
	B = 'B'
	C = 'C'
	D = 'D'

@dataclass
class Target:
	type : PoseTarget
	x : float = 0.0
	y : float = 0.0
	z : float = 0.0
	angle: float = 0.0
	dist : float = 0.0
	
	def set_pose(self, pose):
		x, y, z = pose
		self.x, self.y, self.z = x, y, z
		self.angle = np.arctan2(x, z)
		self.dist = np.sqrt(x**2 + z**2 + y**2)

		# if self.dist > MAX_TRAVEL_DISTANCE: # If distance is too large, the robot should finetune iteratively
		# 	self.dist /= 2
		print(f"Target Distance Set to {self.dist}")
  
class NavigateToPose(Task):
	def __init__(self, target : PoseTarget = PoseTarget.BLUE_BALL):
		super().__init__(name="Navigate")

		self.SPEED = 0.1
		self.TURNRATE = 0.1
		self.ANGLEMARGIN = 0.05
		self.DISTMARGIN = 0.01
		self.OFFSET_BALL = 0.16
		self.OFFSET = 0.2 #Offset 11cm
		self.OFFSET = 0.16 if target == PoseTarget.BLUE_BALL or target == PoseTarget.ORANGE_BALL else 0.2
  
		self.states_q : list[State] = []
		self.state: State = State.LOOKING_FOR_OBJECT
		self.target = Target(target)
		self.finish = False
		self.actions = {
			State.LOOKING_FOR_OBJECT 	: self.get_pose,
			State.TARGET_TURN 			: self.turn_to_target,
			State.TURN_LEFT 			: self.turn_left,
			State.TURN_RIGHT			: self.turn_right,
			State.FORWARD 				: self.forward,
			State.REVERSE				: self.reverse,
			State.CAPTURE 				: self.capture,
		}
  
	def change_state(self) -> None:
		if self.states_q: #Get next state
			self.state = self.states_q.pop(0)
		else: #Default state
			self.state = State.LOOKING_FOR_OBJECT
		LOGGER.debug(f"State: {self.state}, State Queue: {self.states_q}")
		LOGGER.debug(f"Target: {self.target}")
	
	def add_state(self, state) -> None:
		self.states_q.append(state)
  
	def stop(self):
		service.send(service.topicCmd + "ti/rc", "0.0 0.0")
		
	def turn_to_target(self):
		pose.tripBreset()
		if self.target.angle <= -self.ANGLEMARGIN: #If angle is less than margin
			self.add_state(State.TURN_LEFT)
			return False
		elif self.target.angle >= self.ANGLEMARGIN:#If angle is more than margin
			self.add_state(State.TURN_RIGHT)
			return False
		else: #Finished turning
			return True

	def drive_to_target(self):
		pose.tripBreset()
		
		if self.target.dist - self.OFFSET <= -self.DISTMARGIN:
			self.add_state(State.REVERSE)
			return False
		elif self.target.dist - self.OFFSET >= self.DISTMARGIN:
			self.add_state(State.FORWARD)
			return False
		else:
			return True
   

	def turn_left(self):
		self.stop()
		if pose.tripBh >= -self.target.angle: # If done turning
			self.stop()
			self.change_state()
		else:
			self.turn(left=True) # Turn right
	
	def turn_right(self):
		self.stop()
		
		if pose.tripBh <= -self.target.angle: # If done turning
			self.stop()
			self.change_state()
		else:
			self.turn(left=False) # Turn right

	
		

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
   
	def forward(self):
	 	#Calculate missing distance
		if pose.tripB >= self.target.dist - self.OFFSET:
			self.stop()
			self.change_state()
		else:
			self.drive()
	
	def reverse(self):
		#Calculate missing distance
		condition = abs(pose.tripB) >= abs(self.OFFSET - self.target.dist)
		if condition:
			self.stop()
			self.change_state()
		else:
			self.drive(reverse = True)
	
	def capture(self):
		service.send(service.topicCmd + "T0/servo", "1 400 200") # Down position
		# service.send(service.topicCmd + "T0/svos", "0 901 200")
		print('okokokok')
		time.sleep(4)
		self.finish = True
		return 
	
	def get_pose(self) -> bool:
	 
		ok, img, imgTime = cam.getImage() # Get image
		service.send(service.topicCmd + "T0/servo", "1 -1023 200")

  
		# Get pose
		if self.target.type == PoseTarget.BLUE_BALL:
			poses = pose_est_ball_from_img(img, Ball_Color=Ball_Color.BLUE, show=True)
			if not poses: #If no poses turnx
				self.turn()
				return
			
			self.target.set_pose(poses[0]) #Set target
			
		if self.target.type == PoseTarget.ORANGE_BALL:
			poses = pose_est_ball_from_img(img, Ball_Color=Ball_Color.ORANGE, show=True)
			if not poses: #If no poses turnx
				self.turn()
				return
			
			self.target.set_pose(poses[0]) #Set target
			
		if self.target.type == PoseTarget.ARUCO_LD:
			poses = get_pose(img, save_path="aruco_img.png")
			print(poses)
			poses = poses.get(53, None)
			if not poses:
				self.turn()
				return
			rvec, tvec, identifier = poses
			center_pos = drop_point(rvec, tvec, True, offset=-0.03, show = True, img = img)
			print("heya", identifier)
			self.target.set_pose(center_pos)

	        
			
		

		#region Validation constraints
		# VALIDATION STEP 1 TURN
		if not self.turn_to_target(): #Examine if robot needs to turn towards target
			self.change_state()
			return		

		# VALIDATION STEP 2 DRIVE
		elif not self.drive_to_target(): #Examine if robot needs to drive to target or backoff
			self.change_state()
			return
		
		#endregion
		print("GOTTA CAPTURE")
		self.state = State.CAPTURE
		return		

	def loop(self, detection_target: str = "ball"):
		action = self.actions[self.state]
		error = action()
  
		if self.finish:
			return TaskState.SUCCESS

		if error:
			LOGGER.error(f"Failed to execute state action: {self.state} for action {action}")
			return TaskState.FAILURE
	  
		return TaskState.EXECUTING

