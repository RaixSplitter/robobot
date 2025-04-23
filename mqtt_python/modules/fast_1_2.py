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
        
        ### task/job/states
        # self.job = 
        
        ### variables
        # robot
        
        # targets
    
    def debug(self):
        return
        service.send(service.topicCmd + "ti/rc", f"0.0 0.0") # drive straight # speed, angle
        print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        input()
        service.send(service.topicCmd + "ti/rc", self.current_action[0])
    
    def drive(self, action, state : int, set_time : float | None = None, condition : bool = None, enter_to_continue = False):
        # init time
        if self.set_turn_time == None:
            self.set_turn_time = time()
            service.send(service.topicCmd + "ti/rc", action) # speed, angle # call once?
        if condition or (set_time != None and (time() - self.set_turn_time) >= set_time):
            state += 1
            self.set_turn_time = None
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")
            if enter_to_continue:
                input("Press enter")
        return state

    def loop(self):
        self.job()
        return TaskState.EXECUTING
