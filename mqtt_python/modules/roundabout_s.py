from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 
from math import pi

class Roundabout(Task):
    def __init__(self):
        super().__init__(name='roundabout')
        self.topicCmd = "robobot/cmd/" # send to Teensy T0, T1, or teensy_interface
        self.topicRc  = service.topicCmd + "ti/rc"
        # task/job
        # self.job = self.get_to_pos
        self.job = self.do_a_roll
        
        # variables
        self.robot_speed = 0.2
        self.turn_angle  = 0.2 # rad
        
        self.circle_state = 0 # 0 = within range, 1 = out of range
        
        self.object_dist = (0.4, 0.5) # min/max
        self.drive_dist  = 5 # m?

    def get_to_pos(self):
        
        if True:
            # change job
            pose.tripBreset()
            self.job = self.do_a_circle

    def do_a_circle(self):
        if pose.tripB >= self.drive_dist:
            return TaskState.SUCCESS
        sensor_d = ir.ir[0]
        
        if self.circle_state == 0:
            service.send(service.topicCmd + "ti/rc", f"{self.robot_speed} 0.0") # drive straight # speed, angle
            if sensor_d > self.object_dist[0]: # within range
                self.circle_state = 1
        
        elif self.circle_state == 1:
            service.send(service.topicCmd + "ti/rc", f"{self.robot_speed} 0.0") # drive straight # speed, angle
            if sensor_d > self.object_dist[1]: # out of range
                service.send(service.topicCmd + "ti/rc", f"0.1 {self.turn_angle}") # turning # speed, angle
                self.circle_state = 0

    def loop(self):
        self.job()
        return TaskState.EXECUTING
