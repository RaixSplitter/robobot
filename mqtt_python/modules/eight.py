from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class Eight(Task):
    def __init__(self):
        """ Task assumes we are at node 5 facing the hopper """
        super().__init__(name='eight')
        self.action_start_time = None
        
        self.toppling_hopper = True
        self.turning_right = False
        self.driving_into_position = False
        self.found_robot = False
        self.waiting_for_ir = False
        self.follow_line = False
        self.waiting_for_ir_again = False
        self.follow_line_again = False
        
        self.trip_can_be_reset = True
        self.distance_to_hopper = 0.25 # m
        self.right_turn_angle = -3.0 # radians
        self.distance_to_eight = 0.8 + self.distance_to_hopper # m
        self.distances_to_drive_eight = [1.5, 0.9] # m, before and after waiting

    def loop(self):
        """
        Sorry about this function. The flow is:
        - Drive forward to hit the hopper
        - Turn around
        - Drive forward to align us the 8
        - Wait for robot to pass
        - Drive half the 8
        - Wait for robot to pass again
        - Drive quarter of the eight, done        
        """
        if self.action_start_time is None:
            self.action_start_time = time.time()

        if self.trip_can_be_reset:
            pose.tripBreset()
            self.trip_can_be_reset = False

        if self.toppling_hopper:
            # print("Toppling hopper")
            service.send(service.topicCmd + "T0/servo", "1, -500 200") # Medium position
            service.send(service.topicCmd + "ti/rc", "0.3 0.0") # drive forward # speed, angle

            if pose.tripB >= self.distance_to_hopper:
                service.send(service.topicCmd + "T0/servo", "1, -900 200") # Up position
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                self.toppling_hopper = False
                self.turning_right = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

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
            service.send(service.topicCmd + "ti/rc", "0.3 0.0") # drive forward # speed, angle
            if pose.tripB >= self.distance_to_eight:
                self.driving_into_position = False
                self.waiting_for_ir = True
                self.action_start_time = time.time()
                self.trip_can_be_reset = True

        if self.waiting_for_ir:
            # print("Wating for ir")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stand still # speed, angle

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
            edge.Kp, edge.Ki, edge.Kd = (0.9, 0.0, 0.5) # TODO
            edge.set_line_control_targets(
                target_velocity=0.25,
                target_position=1.5 # follow the left edge of the line to assure we take the right turn in the 8
            )
            if self.distances_to_drive_eight[0] < pose.tripB:
                self.follow_line = False
                self.waiting_for_ir_again = True
                self.action_start_time = time.time()
                pose.tripBreset()


        if self.waiting_for_ir_again:
            # print("Wating for ir")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stand still # speed, angle

            distance = ir.ir[1]
            if distance < 0.3 and not self.found_robot:
                # print("self found robot", distance)
                self.found_robot = True
                self.action_start_time = time.time()

            if self.found_robot and 3 < time.time() - self.action_start_time:
                # print("Going")
                self.waiting_for_ir_again = False
                self.follow_line_again = True
                self.action_start_time = time.time()
                pose.tripBreset()

        if self.follow_line_again:
            # print("Following line")
            edge.Kp, edge.Ki, edge.Kd = (0.9, 0.0, 0.5) # TODO
            edge.set_line_control_targets(
                target_velocity=0.25,
                target_position=1.5 # follow the left edge of the line to assure we take the right turn in the 8
            )
            if self.distances_to_drive_eight[1] < pose.tripB:
                return TaskState.SUCCESS


        return TaskState.EXECUTING
