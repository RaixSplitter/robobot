from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from ball_detection import *
from modules.aruco import get_pose, drop_point
from scam import cam
from math import pi
import time 

class NavigateToDropOff(Task):
    def __init__(self):
        self.dont_move = 0.0
        self.drive_fast = 0.75

        self.pose = None  # (x,y,z) x is left/right, y is up/down, z is inwards/outwards
        self
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.z = self.pose[2]
        self.goal_heading = None
        self.length_to_pose = None
        self.distance_from_pose = 0.4
        
        self.trip_has_reset = False
        self.rotate_to_goal_heading = True
        self.has_turned = False
        self.drive_straight_to_pose = False

        self.is_goal_aruco = False
        self.goal_aruco = None
        self.read_aruco = None

    def loop(self):
        if not self.trip_has_reset:
            pose.tripBreset()
            self.trip_has_reset = True
            ok, img, imgTime = cam.getImage()
            self.poses = get_pose(img) # Use Aruco pose estimation function TODO
            for 
            

        # To orient yourself with the goal rotate until heading (h) is:
        # h = arctan(Z/X)
        if self.goal_heading == None and self.length_to_pose == None:
            self.goal_heading = np.arctan2(self.z/self.x)
            self.length_to_pose = np.sqrt(self.x**2+self.y**2)
        
        if not self.is_goal_aruco:
            rotate_to_current_goal_heading()
            is_aruco_drop_off()
            if self.length_to_pose > self.distance_from_pose:
                service.send(service.topicCmd + "ti/rc", "0.2 0.0") # drive straight # speed, angle
                if pose.tripB >= self.length_to_pose - self.distance_from_pose:
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
            if not is_aruco_drop_off():
                navigate_to_different_aruco()

        if self.is_goal_aruco:
            self.drive_straight_to_pose = True

        if self.drive_straight_to_pose:
            service.send(service.topicCmd + "ti/rc", "0.2 0.0") # drive straight # speed, angle
            if pose.tripB >= self.length_to_pose - self.distance_from_pose:
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
            self.drive_straight_to_pose = False
            return TaskState.SUCCESS
        
def rotate_to_current_goal_heading(self):

    if self.rotate_to_goal_heading and not self.has_turned:
            if self.goal_heading < 0:
                service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
                if pose.tripBh <= self.goal_heading:
                    self.has_turned = True
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle

            if self.goal_heading > 0:
                service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
                if pose.tripBh <= self.goal_heading:
                    self.has_turned = True
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle

    return TaskState.EXECUTING

def is_aruco_drop_off(self):
     self.is_goal_aruco = self.goal_aruco == self.read_aruco

def navigate_to_different_aruco(self):
    # Turn Right
    service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
    if pose.tripBh >= -pi/2:
        self.has_turned = True
        service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle

    # Go around the sorting center
    service.send(service.topicCmd + "ti/rc", "0.2 0.2") # turn right # speed, angle
    if pose.tripB >= 0.5:
        self.has_turned = True
        service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle

    # Turn left to face the sorting center
    service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn right # speed, angle
    if pose.tripBh >= pi/4:
        self.has_turned = True
        service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle

    pose.tripBreset()