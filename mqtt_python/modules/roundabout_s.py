from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from time import time 
from math import pi

class Roundabout(Task):
    def __init__(self):
        super().__init__(name='roundabout')
        self.topicCmd = "robobot/cmd/" # send to Teensy T0, T1, or teensy_interface
        self.topicRc  = service.topicCmd + "ti/rc"
        # task/job
        # self.job = self.get_to_pos
        self.job = self.do_a_circle
        
        # variables
        self.robot_speed = 0.1
        self.turn_time   = 0.5 # rad
        self.current_action = [f"{self.robot_speed} 0.0", f"0.0 {self.turn_angle}"]
        
        self.is_turning     = False
        self.set_turn_time  = 0
        
        self.circle_state = 0 # 0 = pre detect, 1 = within range, 2 = out of range
        
        self.object_dist = (0.2, 0.4) # min/max
        self.drive_dist  = 5 # m?
    
    def debug(self):
        return
        service.send(service.topicCmd + "ti/rc", f"0.0 0.0") # drive straight # speed, angle
        print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        input()
        service.send(service.topicCmd + "ti/rc", self.current_action[0])

    def get_to_pos(self):
        # get into position
        ...
        if True:
            # change job
            pose.tripBreset()
            self.job = self.do_a_circle

    def do_a_circle(self):
        if pose.tripB >= self.drive_dist:
            return TaskState.SUCCESS
        
        if self.is_turning:
            if (time() - self.set_turn_time) >= self.turn_angle:
                self.is_turning = False
                self.current_action = list(reversed(self.current_action))
            return TaskState.EXECUTING
            
        service.send(service.topicCmd + "ti/rc", self.current_action[0]) # drive straight # speed, angle
        sensor_d = ir.ir[0]
        
        if self.circle_state == 0:
            if sensor_d < self.object_dist[0]:
                # print("State 0")
                self.circle_state = 1
                self.debug()
        
        elif self.circle_state == 1:
            if sensor_d >= self.object_dist[0] : # within range
                # print("State 1")
                self.circle_state = 2
                self.debug()
        
        elif self.circle_state == 2:
            if sensor_d >= self.object_dist[1]: # out of range
                # print("State 2")
                self.current_action = list(reversed(self.current_action))
                service.send(service.topicCmd + "ti/rc", self.current_action[0]) # turning # speed, angle
                self.is_turning = True
                self.circle_state = 0
                self.set_turn_time = time()
                self.debug()
        

    def loop(self):
        print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        self.job()
        return TaskState.EXECUTING
