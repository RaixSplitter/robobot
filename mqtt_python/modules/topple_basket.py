from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class TOPPLE_BASKET(Task):
    def __init__(self):
        """ Task assumes we are at node 5 facing the hopper """
        super().__init__(name='eight')
        self.action_start_time = None
        
        self.has_initted           = False
        self.toppling_hopper       = False
        self.backing               = False
        self.turning_right         = False
        self.turning_right_again   = False
        self.driving_into_position = False
        self.found_robot           = False
        self.waiting_for_ir        = False
        self.driving_into_line     = False
        self.follow_line           = False
        self.waiting_for_ir_again  = False
        self.follow_line_again     = False
        
        self.trip_can_be_reset = True
        
        # 'Parameters'
        self.pid_values = (2.0, 0.0, 0.6)
        self.move_speed = 0.25
        self.line_target = 0.0 # -2...2 where on the line to follow, we want to lean left to hit crossroad
        self.distance_to_hopper = 0.25 # m
        self.backing_distance   = 0.2 # m
        self.right_turn_angle = 3.1 # radians
        self.distance_to_eight = 0.5 + self.distance_to_hopper - self.backing_distance# m
        self.distances_to_drive_eight = [1.5, 1.1] # m, before and after waiting
        self.wait_times = [3.0, 3.0] # seconds 
        self.detect_robot_distance = 0.3 # meters

    def loop(self):
        """
        Sorry about this function. The flow is:
        - Drive forward to hit the hopper
        - Back a bit
        - Turn around
        - Drive forward to align us the 8
        - Wait for robot to pass
        - Drive half the 8
        - Wait for robot to pass again
        - Drive quarter of the eight, done        
        """
        if not self.has_initted:
            pose.tripBreset()
            self.has_initted = True
            self.turning_right = True
            service.send(service.topicCmd + "T0/servo", "1, -500 200") # Medium position
            self.timer = time.time()
            
        if self.turning_right:
            # print("Turning right")
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn right # speed, angle
            # print("Turning right")
            # if abs(pose.tripBh) >= self.right_turn_angle:
            if (time.time() - self.timer) > 5.0:
                self.turning_right = False
                self.toppling_hopper = True

        if self.toppling_hopper:
            service.send(service.topicCmd + "ti/rc", "0.0 0.0") # drive forward # speed, angle
            input()
            # print("Toppling hopper")
            service.send(service.topicCmd + "ti/rc", "0.25 0.0") # drive forward # speed, angle
            if pose.tripB >= self.distance_to_hopper:
                service.send(service.topicCmd + "T0/servo", "1 -1000 200") # Up position
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                self.toppling_hopper = False
                self.backing = True

        if self.backing:
            # print("Backing")
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            service.send(service.topicCmd + "ti/rc", "-0.2 0.0") # turn right # speed, angle
            if pose.tripB <= self.distance_to_hopper - self.backing_distance:
                self.backing = False
                self.turning_right_again = True
                self.timer = time.time()
                
        if self.turning_right_again:
            # print("Turning right")
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn right # speed, angle
            # print("Turning right")
            # if abs(pose.tripBh) >= self.right_turn_angle:
            if (time.time() - self.timer) > 2.0:
                self.turning_right_again = False
                # return TaskState.SUCCESS_SKIP, 2
                return TaskState.SUCCESS


        return TaskState.EXECUTING
