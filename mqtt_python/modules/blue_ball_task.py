import time

from modules.task import Task, TaskState
from modules.navigate_to_pose import NavigateToPose, PoseTarget
from sir import ir
from sedge import edge
from spose import pose
from uservice import service

class BlueBallTask(Task):
	def __init__(self):
		"""
		Assumes we start at node 6 and a clockwise 90 deg turn has been made
		"""
		super().__init__(name='blue_ball_task')

		self.navigator = NavigateToPose(target = PoseTarget.BLUE_BALL)

		self.max_search_time = 15 # seconds
		self.start_search_time = None

		# flags
		self.task_started = False
		self.moving_to_ball = False
		self.returning_to_line = False
		self.has_failed = False

	def loop(self):
		if not self.task_started:
			self.task_started = True
			self.moving_to_ball = True
			self.start_search_time = time.time()
			pose.tripBreset()
		
		if self.moving_to_ball:
			task_state = self.navigator.loop()

			if task_state == TaskState.SUCCESS:
				self.moving_to_ball = False
				self.returning_to_line = True
			
			elif task_state == TaskState.FAILURE:
				self.moving_to_ball = False
				self.returning_to_line = True
				self.has_failed = True
			
			elif task_state == TaskState.LOST:
				self.moving_to_ball = False
				self.returning_to_line = True
				self.has_failed = True

			# Have we searched for long without resolve we restart
			if self.max_search_time < time.time() - self.start_search_time:
				self.has_failed = True
		
		# Navigate back to safety
		if self.returning_to_line:
			pass

		# Would like to avoid this, and even so if we should move to safety beforehand
		if self.has_failed:
			return TaskState.FAILURE

		return TaskState.EXECUTING
