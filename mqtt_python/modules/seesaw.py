import time

from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose

class Seesaw(Task):
	def __init__(self):
		super().__init__(name='seesaw')

		self.task_started = False
		self.moving_to_ball = False
		self.ball_picked_up = False
		self.dist_to_ball = 0.4 # m

	def loop(self):
		if not self.task_started:
			self.task_started = True
			self.moving_to_ball = True
			pose.tripBreset()
		
		# Move to ball
		if self.moving_to_ball and self.dist_to_ball <= pose.tripB:
			service.send(service.topicCmd + "T0/servo", f"1 250 1000")
			time.sleep(10)

