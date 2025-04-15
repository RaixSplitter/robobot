from modules.task import Task, TaskState
import numpy as np
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from modules.ball_detection import *
from modules.aruco import get_pose, drop_point
from scam import cam
from math import pi
import time 

class NavigateToDropOff(Task):
    def __init__(self):
        self.dont_move = 0.0
        self.drive_fast = 0.75
        self.states = {
            "Reset trip B": 0,
            "Set goal heading/length": 1,
            "Current aruco is not goal": 2,
            "Current aruco is goal": 3
        }
        self.state = 0
        self.pose = None  # (x,y,z) x is left/right, y is up/down, z is inwards/outwards
        self.pose_distances = []
        self.aruco_poses = []
        self.x = None
        self.y = None
        self.z = None
        self.goal_heading = None
        self.length_to_pose = None
        self.distance_from_pose = 0.4
        
        self.trip_has_reset = False
        self.rotate_to_goal_heading = True
        self.has_turned = False
        self.drive_straight_to_pose = False
        self.has_navigated_to_different_aruco = False
        self.has_faced_center = False
        self.has_turned_90 = False
        self.has_gone_around = False

        self.is_goal_aruco = False
        self.goal_aruco = None # blue ball aruco
        self.read_aruco = None
        self.goal_ids = [14,15]

    def loop(self):
        if self.state == 0:
            pose.tripBreset()
            self.trip_has_reset = True
            self.state = self.get_new_aruco_poses()

        # To orient yourself with the goal rotate until heading (h) is:
        # h = arctan(Z/X)
        if self.state == 1:
            self.goal_heading = np.arctan2(self.x, self.z)
            self.length_to_pose = np.sqrt(self.x**2+self.z**2)
        
        self.is_aruco_drop_off()
        if self.state == 2:
            self.rotate_to_current_goal_heading()
            if self.length_to_pose >= self.distance_from_pose:
                service.send(service.topicCmd + "ti/rc", "0.2 0.0") # drive straight # speed, angle
                if pose.tripB >= self.length_to_pose - self.distance_from_pose:
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                    pose.tripBreset()
            if not self.is_aruco_drop_off():
                self.navigate_to_different_aruco()
                if self.has_navigated_to_different_aruco:
                    self.get_new_aruco_poses()

        if self.state == 3:
            service.send(service.topicCmd + "ti/rc", "0.2 0.0") # drive straight # speed, angle
            if pose.tripB >= self.length_to_pose - self.distance_from_pose:
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                pose.tripBreset()
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
        self.is_goal_aruco = self.read_aruco in self.goal_ids
        if self.is_goal_aruco:
            self.state = self.states["Current aruco is goal"]
        else:
            self.state = self.states["Current aruco is not goal"]
        return self.is_goal_aruco

    def navigate_to_different_aruco(self):
        # Turn Right
        if not self.has_turned_90:
            service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
            # 90 grader
            if abs(pose.tripBh) >= pi/2:
                self.has_turned_90 = True
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                # Reset trip to track later
                pose.tripBreset()
        if self.has_turned_90 and not self.has_gone_around:
            # Go around the sorting center
            service.send(service.topicCmd + "ti/rc", "0.2 0.2") # turn right # speed, angle
            # Cirka 50 cm
            if pose.tripB >= 0.5:
                self.has_gone_around = True
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                # Reset trip to track later
                pose.tripBreset()
                self.has_gone_around = True
        if self.has_turned_90 and self.has_gone_around and not self.has_faced_center:
            # Turn left to face the sorting center
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn right # speed, angle
            # 45 degrees
            if pose.tripBh >= pi/4:
                self.has_faced_center = True
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                # Reset trip to track later
                pose.tripBreset()
        

    def get_new_aruco_poses(self):
        ok, img, imgTime = cam.getImage()
        self.aruco_poses = get_pose(img) # Use Aruco pose estimation function TODO
        for aruco_pose_key in self.aruco_poses.items():
            aruco_pose = drop_point(self.aruco_poses[aruco_pose_key])
            distance = np.linalg.norm(aruco_pose)
            self.pose_distances.append((aruco_pose_key,distance))
        closest_id, _ = min(self.pose_distances, key=lambda x: x[1])
        self.read_aruco = closest_id
        self.pose = drop_point(self.aruco_poses[closest_id])
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.z = self.pose[2]

        return self.states["Set goal heading/length"]