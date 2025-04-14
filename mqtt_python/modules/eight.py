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
        
        self.turning_right = True
        self.driving_into_position = False
        self.found_robot = False
        self.waiting_for_ir = False
        self.follow_line = False
        
        self.trip_can_be_reset = True
        self.right_turn_angle = -3.0 # radians
        self.distance_to_eight = 0.8 # m
        self.distance_to_drive_eight = 2.5 # m

    def loop(self):
        if self.action_start_time is None:
            self.action_start_time = time.time()

        if self.trip_can_be_reset:
            pose.tripBreset()
            self.trip_can_be_reset = False

        if self.turning_right:
            # print("Turning right")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 -0.8") # turn right # speed, angle
            if pose.tripBh <= self.right_turn_angle:
                self.turning_right = False
                self.driving_into_position = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

        if self.driving_into_position:
            # print("Driving straight")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.3 0.0") # turn right # speed, angle
            if pose.tripB >= self.distance_to_eight:
                self.driving_into_position = False
                self.waiting_for_ir = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

        if self.waiting_for_ir:
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0") # turn right # speed, angle

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
            edge.Kp, edge.Ki, edge.Kd = (0.9, 0.0, 0.5) # TODO
            edge.set_line_control_targets(
                target_velocity=0.25,
                target_position=0.0
            )
            if self.distance_to_drive_eight < pose.tripB:
                return TaskState.SUCCESS


        return TaskState.EXECUTING
