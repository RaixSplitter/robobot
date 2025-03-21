from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class Eight(Task):
    def __init__(self):
        super().__init__(name='eight')
        self.action_start_time = None
        
        self.turning_left = True
        self.driving_into_position = False
        self.found_robot = False
        self.waiting_for_ir = False
        self.follow_line = False
        

    def loop(self):
        if self.action_start_time is None:
            self.action_start_time = time.time()

        if self.turning_left:
            # print("Turning left")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
            if 2.3 < time.time() - self.action_start_time:
                self.turning_left = False
                self.driving_into_position = True
                self.action_start_time = time.time()

        if self.driving_into_position:
            # print("Driving straight")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.2 0.0") # turn left # speed, angle
            if 2.0 < time.time() - self.action_start_time:
                self.driving_into_position = False
                self.waiting_for_ir = True
                self.action_start_time = time.time()

        if self.waiting_for_ir:
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0") # turn left # speed, angle

            distance = ir.ir[1]
            if distance < 0.3 and not self.found_robot:
                # print("self found robot", distance)
                self.found_robot = True
                self.action_start_time = time.time()

            if self.found_robot and 3 < time.time() - self.action_start_time:
                # print("Going")
                self.waiting_for_ir = False
                self.follow_line = True
                self.action_start_time = time.time()
                pose.tripBreset()

        if self.follow_line:
            # print("Following line")

            edge.set_line_control_targets(
                target_velocity=0.2,
                target_position=0.0
            )
            if 2.75 < pose.tripB:
                return TaskState.SUCCESS


        return TaskState.EXECUTING
