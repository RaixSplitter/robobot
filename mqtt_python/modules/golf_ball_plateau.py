from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from math import pi
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

		self.angle_to_face_ball = 3*-pi/4 # 135 grader
		self.tiny_angle = pi/12 # 15 grader
		self.has_turned_to_face_ball = False
		self.ball_is_in_view = False

		self.lower_arm = False
		self.turn_around = False
		self.turn_around_angle = -3
		self.has_turned_around = False

		self.length_to_hole = .5
		self.wiggle = False

	def loop(self, detection_target: str = 'aruco'):
		if not self.trip_has_reset:
			pose.tripBreset()
			self.trip_has_reset = True
		
		if not self.has_turned_to_face_ball:
			service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
			# 135 grader
			if pose.tripBh <= self.angle_to_face_ball:
				self.has_turned_to_face_ball = True
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				# Reset trip to track later
				pose.tripBreset()

		if not self.ball_is_in_view: 
			ok, img, imgTime = cam.getImage()
			# found_poses = pose_est_ball_from_img(img) # (x,y,z) x is left/right, y is up/down, z is inwards/outwards		
			found_poses = pose_est_ball_from_img(img)
			
			if len(found_poses) == 0:
				self.trip_has_reset = False
				print("Error: didnt find any objects in task")
				self.ball_is_in_view = False

			if len(found_poses) > 0:
				self.x = found_poses[0][0]
				self.y = found_poses[0][1]
				self.z = found_poses[0][2]
				self.ball_is_in_view = True
			
			if not self.ball_is_in_view:
				service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
				# 135 grader
				if pose.tripBh >= self.angle_to_face_ball:
					self.has_turned_to_face_ball = True
					service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
					# Reset trip to track later
					pose.tripBreset()

			# print(f"Set goal pose: {self.pose}, x,y,z: {self.x, self.y, self.z}")
		# To orient yourself with the ball rotate until heading (h) is:
		# h = arctan(Z/X)

		if self.goal_heading == None and self.length_to_pose == None and self.ball_is_in_view:
			self.goal_heading = np.arctan2(self.x, self.z)
			self.length_to_pose = np.sqrt(self.x**2+self.z**2)
			print("Heading to goal:", self.goal_heading, "Distance to goal:", self.length_to_pose)
		
		if self.rotate_to_goal_heading and not self.has_turned and self.goal_heading != None:
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
				self.lower_arm = True
				pose.tripBreset()

		
		if self.lower_arm:
			service.send(service.topicCmd + "T0/servo", f"1 250 1000")
			self.turn_around = True
			self.lower_arm = False
		
		if self.turn_around:
			service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
			# 135 grader
			if pose.tripBh <= self.turn_around_angle:
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				# Reset trip to track later
				pose.tripBreset()
				self.turn_around = False
				self.has_turned_around = True

		if self.has_turned_around:
			service.send(service.topicCmd + "ti/rc", "0.1 0.0") # drive straight # speed, angle
			print(f"{pose.tripB:.2f}, {self.length_to_pose}")
			if pose.tripB >= self.length_to_hole:
				service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
				self.drive_straight_to_pose = False
				self.lower_arm = True
				self.has_turned_around = False
				pose.tripBreset()
				self.wiggle = True

		if self.wiggle:
			##### Wiggle function 
			# which sets check if ball 
			# and wiggle to false

			if self.check_if_ball:

				if self.ball_still_in_view:
					self.increment_forward

		if self.increment_forward:

			self.wiggle = True
			
		return TaskState.EXECUTING