from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class Roundabout(Task):
    def __init__(self):
        super().__init__(name='roundabout')
        self.turn_start = None
        self.turn_rate = 0.6 # turn left
        self.turn_speed = 0.2

        self.circling = False

    def loop(self):
        if self.turn_start is None:
            self.turn_start = time.time()
            pose.tripBreset()
        
        if 3 < pose.tripB:
            print("Time turning: ", time.time() - self.turn_start)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")    
            return TaskState.SUCCESS


        return TaskState.EXECUTING
