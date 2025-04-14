import time
import math

from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service

FPS = 20 # this is set in mqtt, we lock at 20 fps

class DeliverGolfBall(Task):
	def __init__(self):
		"""
		We assume that this state is called from 'node 10' in Sonny's map and the arm is down and has a ball
		""" 
		super().__init__(name='deliver_golf_ball')
		self.dist_to_hole = 0.2 # m

		self.move_speed = 0.25
		self.speed_when_wiggling = 0.05

		self.frames_to_wiggle = 20 # how many time steps does a sweep from left to right take
		
		self.time_to_take_up_arm = 2.0 # seconds
		self.time_to_wiggle = 2.0 # seconds
		
		self.put_arm_up_time = None
		self.wiggle_start_time = None
		self.turn_amount = 1 # oscillates between -1 and 1

		# flags
		self.task_started = False
		self.moving_to_hole = False
		self.wiggling = False
		self.resetting_pose = False

		self.i = int(self.frames_to_wiggle / 2) # 'time' step, used for wiggle


	def loop(self):
		self.i += 1

		if not self.task_started:
			self.task_started = True
			self.moving_to_hole = True
			service.send(service.topicCmd + "T0/servo", "0 199 3000") # force servo down in case
			pose.tripBreset()
		
		# Move to hole
		if self.moving_to_hole:
			edge.Kp, edge.Ki, edge.Kd = (0.9, 0.0, 0.5) # TODO
			edge.set_line_control_targets(
				target_velocity=self.move_speed,
				target_position=0.0
			)
			if self.dist_to_hole <= pose.tripB:
				edge.set_line_control_targets(0,0)
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # speed, angle
				self.moving_to_hole = False
				self.wiggling = True
				self.wiggle_start_time = time.time()
			
		# Wiggle around
		if self.wiggling:
			# Every n frames switch between wiggling from left to right
			if self.i % self.frames_to_wiggle == 0:
				self.turn_amount *= -1

			service.send(service.topicCmd + "ti/rc", f"{self.speed_when_wiggling} {self.turn_amount}") # speed, angle

			# Wiggle for a few seconds
			if self.wiggle_start_time and self.time_to_wiggle < time.time() - self.wiggle_start_time:
				service.send(service.topicCmd + "ti/rc", f"0.0 0.0") # speed, angle
				self.put_arm_up_time = time.time()
				self.wiggling = False

		# Check of we are done delivering ball, if so done
		if self.put_arm_up_time and self.time_to_take_up_arm < time.time() - self.put_arm_up_time:
			service.send(service.topicCmd + "T0/servo", "0 -900 3000")
			self.put_arm_up_time = None # dumb reset
			self.resetting_pose = True

		# Reset pose to original
		if self.resetting_pose:
			if abs(pose.tripBh) < 0.05:
				return TaskState.SUCCESS
			elif 0 < pose.tripBh:
				service.send(service.topicCmd + "ti/rc", f"0.0 -0.1") # speed, angle
			else:
				service.send(service.topicCmd + "ti/rc", f"0.0 0.1") # speed, angle

		return TaskState.EXECUTING
