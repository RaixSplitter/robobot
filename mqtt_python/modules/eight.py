from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
from math import pi
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
        
        self.trip_can_be_reset = True
        self.left_turn_angle = pi / 2 # 90 degrees
        self.distance_to_eight = .3 # 30 cm to figure eight

    def loop(self):
        if self.action_start_time is None:
            self.action_start_time = time.time()

        if self.trip_can_be_reset:
            pose.tripBreset()
            self.trip_can_be_reset = False

        if self.turning_left:
            # print("Turning left")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
            if pose.tripBh >= self.left_turn_angle:
                self.turning_left = False
                self.driving_into_position = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

        if self.driving_into_position:
            # print("Driving straight")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.2 0.0") # turn left # speed, angle
            if pose.tripB >= self.distance_to_eight:
                self.driving_into_position = False
                self.waiting_for_ir = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

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
