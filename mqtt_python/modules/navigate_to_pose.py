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

class NavigateToPose(Task):
	def __init__(self):
		super().__init__(name='Navigate')
		self.dont_move = 0.0
		self.drive_fast = 0.75

		self.x = None
		self.y = None
		self.z = None
		self.goal_heading = None
		self.length_to_pose = None
		self.distance_from_pose = 0.16 # meters
		
		self.trip_has_reset = False
		self.rotate_to_goal_heading = True
		self.has_turned = False
		self.drive_straight_to_pose = False

	def loop(self, detection_target: str = 'aruco'):
		if not self.trip_has_reset:
			pose.tripBreset()
			self.trip_has_reset = True
			ok, img, imgTime = cam.getImage()
			# found_poses = pose_est_ball_from_img(img) # (x,y,z) x is left/right, y is up/down, z is inwards/outwards
			if detection_target == 'aruco':
				poses_dict = get_pose(img)
				_, found_pose = list(poses_dict.values())[0]
				found_poses = [found_pose.flatten()]
				print("LSMLKASDL", found_poses)

			elif detection_target == 'ball':
				found_poses = pose_est_ball_from_img(img)
			else:
				raise ValueError()

			if len(found_poses) == 0:
				self.trip_has_reset = False
				print("Error: didnt find any objects in task")
				return TaskState.FAILURE
			
			self.x = found_poses[0][0]
			self.y = found_poses[0][1]
			self.z = found_poses[0][2]
			# print(f"Set goal pose: {self.pose}, x,y,z: {self.x, self.y, self.z}")
		# To orient yourself with the ball rotate until heading (h) is:
		# h = arctan(Z/X)

		if self.goal_heading == None and self.length_to_pose == None:
			self.goal_heading = np.arctan2(self.x, self.z)
			self.length_to_pose = np.sqrt(self.x**2+self.z**2)
			print("Heading to goal:", self.goal_heading, "Distance to goal:", self.length_to_pose)
		
		if self.rotate_to_goal_heading and not self.has_turned:
			if self.goal_heading < 0:
				# Turn Left?
				if -pose.tripBh <= self.goal_heading:
					print("Done turning left")
					self.has_turned = True
					service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				else:
					service.send(service.topicCmd + "ti/rc", "0.0 0.1") # turn left # speed, angle

			if self.goal_heading > 0:
				# Turn Right? 
				if abs(pose.tripBh) >= self.goal_heading:
					print("Done turning right")
					self.has_turned = True
					service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				else:
					service.send(service.topicCmd + "ti/rc", "0.0 -0.1") # turn right # speed, angle
					
			if self.has_turned:
				print("Switching to driving straight")
				self.drive_straight_to_pose = True

		if self.drive_straight_to_pose:
			service.send(service.topicCmd + "ti/rc", "0.1 0.0") # drive straight # speed, angle
			print(f"{pose.tripB:.2f}, {self.length_to_pose}")
			if pose.tripB >= self.length_to_pose - self.distance_from_pose:
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				self.drive_straight_to_pose = False
				service.send(service.topicCmd + "T0/servo", "1 230 200") # down position
				return TaskState.SUCCESS
			
		return TaskState.EXECUTING