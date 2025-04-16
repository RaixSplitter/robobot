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
        
        ### variables
        # robot
        self.robot_speed = 0.1
        self.turn_time   = 0.5 # rad
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.4
        
        # state
        self.circle_state = 0 # 0 = pre detect, 1 = within range, 2 = out of range
        self.current_action = [f"{self.robot_speed} 0.0", f"0.0 {self.turn_angle}"]
        self.is_turning     = False
        self.set_turn_time  = 0
        self.dt             = max(ir.irInterval / 1000.0, 1e-6) # Avoid extremely small dt    
        
        # targets
        self.object_dist = (0.2, 0.4) # min/max
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
        
    def do_a_simple_circle(self):
        if pose.tripB >= self.drive_dist:
            return TaskState.SUCCESS
                
        sensor_d = ir.ir[0]
        if sensor_d != 1.5:
            service.send(service.topicCmd + "ti/rc", self.current_action[0]) # drive straight # speed, angle
        else:
            service.send(service.topicCmd + "ti/rc", self.current_action[1]) # turning        # speed, angle

        

    def loop(self):
        print("circle state:", self.circle_state, "ir distance:", ir.ir[0], "bh:", pose.tripBh)
        self.job()
        return TaskState.EXECUTING
