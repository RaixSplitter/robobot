import time

from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service

class Seesaw(Task):
	def __init__(self):
		super().__init__(name='seesaw')

		self.move_speed = 0.25
		self.dist_to_ball = 1.2 # m
		self.time_to_put_down_arm = 2.0 # seconds
		self.put_arm_down_start = None

		# flags
		self.task_started = False
		self.moving_to_ball = False

	def loop(self):
		if not self.task_started:
			self.task_started = True
			self.moving_to_ball = True
			pose.tripBreset()
			print("RESET")
		
		# Move to ball
		if self.moving_to_ball:
			edge.Kp, edge.Ki, edge.Kd = (0.9, 0.0, 0.5) # TODO
			edge.set_line_control_targets(
				target_velocity=self.move_speed,
				target_position=0.0
			)
			print(pose.tripB)
			if self.dist_to_ball <= pose.tripB:
				print("putting arm down")
				edge.set_line_control_targets(0,0)
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # speed, angle
				service.send(service.topicCmd + "T0/servo", "1 250 3000")
				self.moving_to_ball = False
				self.put_arm_down_start = time.time()
		
		# Check of we have picked up ball, if so done
		if self.put_arm_down_start and self.time_to_put_down_arm < time.time() - self.put_arm_down_start:
			service.send(service.topicCmd + "T0/servo", "0 200 3000")
			return TaskState.SUCCESS

		return TaskState.EXECUTING
