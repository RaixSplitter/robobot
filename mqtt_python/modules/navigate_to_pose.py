from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from ball_detection import *
import time 

class NavigateToPose(Task):
    def __init__(self):
        super().__init__(name='Navigate')
        self.dont_move = 0.0
        self.drive_fast = 0.75

        self.pose = None  # (x,y,z) x is left/right, y is up/down, z is inwards/outwards
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.z = self.pose[2]
        self.goal_heading = None
        self.length_to_pose = None
        self.distance_from_pose = 0.1
        
        self.trip_has_reset = False
        self.rotate_to_goal_heading = True
        self.has_turned = False
        self.drive_straight_to_pose = False

    def navigate(self):
        if not self.trip_has_reset:
            pose.tripBreset()
            self.trip_has_reset = True
            self.pose = pose_est_ball_from_img(image)

        # To orient yourself with the ball rotate until heading (h) is:
        # h = arctan(Z/X)

        if self.goal_heading == None and self.length_to_pose == None:
            self.goal_heading = np.arctan2(self.z/self.x)
            self.length_to_pose = np.sqrt(self.x**2+self.y**2)
        
        if self.rotate_to_goal_heading and not self.has_turned:
            if self.goal_heading < 0:
                # Turn Left?
                service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
                if pose.tripBh <= self.goal_heading:
                    self.has_turned = True
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # turn left # speed, angle

            if self.goal_heading > 0:
                # Turn Right? 
                service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn left # speed, angle
                if pose.tripBh <= self.goal_heading:
                    self.has_turned = True
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # turn left # speed, angle

            if self.has_turned:
                self.drive_straight_to_pose = True

        if self.drive_straight_to_pose:
            service.send(service.topicCmd + "ti/rc", "0.2 0.0") # turn left # speed, angle
            if pose.tripB >= self.length_to_pose - self.distance_from_pose:
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # turn left # speed, angle
            self.drive_straight_to_pose = False
            return TaskState.SUCCESS