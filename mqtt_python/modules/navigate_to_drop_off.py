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
from modules.navigate_to_pose import PoseTarget

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
		self.dist = np.sqrt(x**2 + z**2)

		if self.dist > MAX_TRAVEL_DISTANCE: # If distance is too large, the robot should finetune iteratively
			self.dist /= 2
		print(f"Target Distance Set to {self.dist}")
  
class NavigateBallCenter(Task):
	def __init__(self, target : PoseTarget = PoseTarget.BLUE_BALL):
		super().__init__(name="Navigate")

		self.SPEED = 0.1
		self.TURNRATE = 0.1
		self.ANGLEMARGIN = 0.05
		self.DISTMARGIN = 0.01
		self.OFFSET = 0.16 #Offset 11cm
		
  
		self.states_q : list[State] = []
		self.state: State = State.LOOKING_FOR_BALLCENTER
		self.default_state : State = State.LOOKING_FOR_BALLCENTER
		self.target = Target(target)
		self.finish = False
		self.actions = {
			State.LOOKING_FOR_BALLCENTER 	: self.looking_for_ballcenter,
			State.VERIFY_POSITION 		    : self.verify_position,
			State.NAVIGATE_TO_NEXT_CORNER 	: self.navigate_to_next_corner,
			State.TARGET_TURN 			    : self.turn_to_target,
			State.TURN_LEFT 			    : self.turn_left,
			State.TURN_RIGHT			    : self.turn_right,
			State.FORWARD 				    : self.forward,
			State.REVERSE				    : self.reverse,
			State.DELIVER 				    : self.deliver,
		}
	
	#region helper functions
 	def loop(self, detection_target: str = "ball"):
		action = self.actions[self.state]
		error = action()
  
		if self.finish:
			return TaskState.SUCCESS

		if error:
			LOGGER.error(f"Failed to execute state action: {self.state} for action {action}")
			return TaskState.FAILURE
	  
		return TaskState.EXECUTING
	def change_state(self) -> None:
		if self.states_q: #Get next state
			self.state = self.states_q.pop(0)
		else: #Default state
			self.state = self.default_state
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
		print(condition, pose.tripB, self.target.dist, self.OFFSET)
		if condition:
			self.stop()
			self.change_state()
		else:
			self.drive(reverse = True)
	
	def deliver(self):
		service.send(service.topicCmd + "T0/servo", "1, -900 1") # Up position
		self.finish = True
		return 
	#endregion
	
	def looking_for_ballcenter(self):
		pass

	def verify_position(self):
		ok, img, imgTime = cam.getImage() # Get image
		service.send(service.topicCmd + "T0/servo", "1 -901 200")

  
		# Get pose
		if self.target.type == PoseTarget.BLUE_BALL:
			poses = pose_est_ball_from_img(img, Ball_Color=Ball_Color.ORANGE)
		if self.target.type == PoseTarget.ARUCO:
			poses = get_pose(img, "captured_image.jpg")
			# Filter poses to only include keys 'A', 'B', 'C', and 'D'
			poses = {key: value for key, value in poses.items() if key in {'A', 'B', 'C', 'D'}}
			
		if not poses: #If no poses turn
			self.turn()
			return
		
		self.target.set_pose(poses[0]) #Set target

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

		self.state = State.CAPTURE
		return		

