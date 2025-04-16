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


class RetrieveLuggage(Task):
	def __init__(self):
		super().__init__(name="Navigate")
		#region CONFIG
		self.SPEED = 0.1
		self.DEFAULT_STATE = State.INIT
		self.DIST_TO_WALL = 0.05 #[M]
		#endregion
  
		#region States
		self.states_q : list[State] = []
		self.state: State = State.WAIT
		self.finish = False
		self.actions = {
			State.INIT 			: self.get_into_position,
			State.LOWER_ARM 	: self.lower_arm,
			State.WAIT 			: self.wait,
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

	def get_into_position(self):
		service.send(service.topicCmd + "T0/servo", "1, 0 1") # Up position
		
		distance = ir.ir[1] #IR distance to object in front
		
		if distance > self.DIST_TO_WALL:
			self.drive()
			return
		else:
			self.stop()
			self.add_state(State.WAIT)
			return
			

	def get_into_position(self):
		self.stop()
		pass


