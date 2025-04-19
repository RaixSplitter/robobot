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
        self.job = self.do_a_outer_circle
        
        ### variables
        # robot
        self.robot_speed = 0.1
        self.turn_time   = 1.5 # s
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.4
        
        # state
        self.circle_state = 0 # 0 = pre detect, 1 = within range, 2 = out of range
        # self.current_action = [f"{self.robot_speed} 0.0", f"{self.robot_speed} 0.30", f"0.0 0.7"] # do_a_circle
        # self.current_action = [f"{self.robot_speed} 0.0", f"{self.robot_speed} 0.25", f"0.0 0.5"] # do_a_circle
        self.current_action = [f"{self.robot_speed} 0.0", f"{self.robot_speed} 0.25", f"0.0 0.5"] # do_a_circle
        
        self.is_turning     = False
        self.set_turn_time  = 0
        self.dt             = max(ir.irInterval / 1000.0, 1e-6) # Avoid extremely small dt    
        self.prev_d         = 2.0
        
        # targets
        self.object_dist = (0.2, 0.4) # min/max
        self.outer_min_dist = 0.2
        self.drive_dist  = 5 # m?
    
    def debug(self):
        return
        service.send(service.topicCmd + "ti/rc", f"0.0 0.0") # drive straight # speed, angle
        print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        input()
        service.send(service.topicCmd + "ti/rc", self.current_action[0])

    def pid_loop(self, target: float, current: float) -> float:
        error = target - current
        self.cumulative_error += error * self.dt
        # Potentially clamp integrator more tightly or differently
        self.cumulative_error = max(min(self.cumulative_error, 1.0), -1.0)

        diff_error = (error - self.previous_error) / self.dt
        self.previous_error = error

        u = (self.Kp * error) \
          + (self.Ki * self.cumulative_error) \
          + (self.Kd * diff_error)
        # print(f"target:{target:.2f}, current:{current:.2f}, e:{error:.2f}, u:{u:.2f}, u1:{max(min(u, 1.0), -1.0):.2f}")
        return max(min(u, 0.35), -0.35)

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
                    
        service.send(service.topicCmd + "ti/rc", self.current_action[self.circle_state]) # drive straight # speed, angle
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
            if self.is_turning:
                if (time() - self.set_turn_time) >= self.turn_time:
                    self.is_turning = False
                    self.circle_state = 0
                    # self.current_action = list(reversed(self.current_action))
                return TaskState.EXECUTING
            if sensor_d >= self.object_dist[1]: # out of range
                # print("State 2")
                # self.current_action = list(reversed(self.current_action))
                # service.send(service.topicCmd + "ti/rc", self.current_action[self.circle_state]) # turning # speed, angle
                self.is_turning = True
                self.set_turn_time = time()
                self.debug()
        
    def do_a_simple_circle(self):
        minv = 0.15
        maxv = 1.30
        if pose.tripB >= self.drive_dist:
            return TaskState.SUCCESS
                
        sensor_d = ir.ir[0]
        if minv <= sensor_d and sensor_d <= maxv:
            service.send(service.topicCmd + "ti/rc", f"0.15 {0.40 + 0.05 * (1-(sensor_d-minv)/(maxv-minv))}") # turning        # speed, angle
            print("This", 0.40 + 0.05 * (1-(sensor_d-minv)/(maxv-minv)))
            pass
        else:
            service.send(service.topicCmd + "ti/rc", "0.15 0.0") # drive straight # speed, angle
            pass

    def do_a_outer_circle(self):
        if pose.tripB >= self.drive_dist:
            return TaskState.SUCCESS
                    
        sensor_d = ir.ir[0]
        action = "0.15 0.0"
        
        if self.is_turning:
            action = "0.0 0.40"
            if (time() - self.set_turn_time) >= self.turn_time or sensor_d >= self.outer_min_dist:
                print("Turning off", (time() - self.set_turn_time) >= self.turn_time, sensor_d >= self.outer_min_dist)
                self.is_turning = False
                self.prev_d = 2.0
        
        elif sensor_d > self.prev_d:
            # print("Turning on")
            action = "0.0 0.40"
            self.is_turning = True
            self.set_turn_time = time()
        self.prev_d = sensor_d
        
        service.send(service.topicCmd + "ti/rc", action) # speed, angle        

    def loop(self):
        # print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        print("ir distance:", ir.ir[0], ir.ir[0] >= self.outer_min_dist)
        self.job()
        return TaskState.EXECUTING
