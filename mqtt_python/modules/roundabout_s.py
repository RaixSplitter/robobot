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
        self.job = self.get_to_pos
        # self.job = self.do_a_circle
        # self.job = self.do_a_const_circle
        # self.job = self.get_out
        
        self.pos_state    = 0
        # self.circle_state = 0 # unused
        self.exit_state   = 0
        
        ### variables
        # robot
        
        # self.current_action = [f"{self.robot_speed} 0.0", f"{self.robot_speed} 0.25", f"0.0 0.5"] # do_a_circle
        
        self.set_turn_time  = None
        self.exit_timer     = None
        # self.dt             = max(ir.irInterval / 1000.0, 1e-6) # Avoid extremely small dt
        # self.prev_d         = 99
        # self.saved_angle    = -100
        # self.angle_changed  = False
    
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

    def get_to_pos(self):
        # sensor_d = ir.ir[0] # Error
        print("Running")
        if self.pos_state == -1:
            print("This Pos")
            input()
            self.pos_state = 0
        
        # get into position
        elif self.pos_state == 0:
            self.pos_state = self.drive("0.45 1.5", self.pos_state, set_time = 1.8)
        
        elif self.pos_state == 1: 
            self.pos_state = self.drive("-0.3 -0.1", self.pos_state, set_time = 2.0)
            
        elif self.pos_state == 2: 
            self.pos_state = self.drive("-0.3 0.1", self.pos_state, set_time = 1.0)

        elif self.pos_state == 3:
            self.pos_state = self.drive("0.35 -0.3", self.pos_state, set_time = 2.5)
            
        elif self.pos_state == 4:
            self.pos_state = self.drive("0.0 -1.5", self.pos_state, set_time = 0.4)
            # if self.set_turn_time == None:
            #     self.set_turn_time = time()
            # if (time() - self.set_turn_time) >= 1.0 and sensor_d <= 0.4:
            #     self.pos_state += 1
            #     self.set_turn_time = None
            # service.send(service.topicCmd + "ti/rc", "0.0 -1.0") # turn # speed, angle

        # elif self.pos_state == 4:
        #     self.pos_state = self.drive("0.0 1.0", self.pos_state, set_time = 0.3)
            
        elif self.pos_state == 5:
            # change job
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")
            # pose.tripBreset()
            # self.saved_angle = pose.tripBh
            # self.job = self.do_a_circle
            self.job = self.do_a_const_circle

    ### Circle testing ###
    # def do_a_circle(self, outer_min_dist = 0.1):
    #     self.check_if_out()
                    
    #     # sensor_d = ir.ir[0]
    #     action = "0.20 0.0"
        
    #     if self.is_turning:
    #         action = "0.20 1.1"
    #         if self.set_turn_time == None and sensor_d >= outer_min_dist:
    #             self.set_turn_time = time()
    #         if self.set_turn_time != None and (time() - self.set_turn_time) >= 0.5:
    #             print("Turning off", sensor_d >= outer_min_dist)
    #             self.is_turning = False
    #             self.set_turn_time = None
    #             self.prev_d = 99
        
    #     elif sensor_d > self.prev_d and sensor_d < 0.5:
    #         # print("Turning on")
    #         # action = "0.15 1.0"
    #         self.is_turning = True
    #     self.prev_d = sensor_d
        
    #     service.send(service.topicCmd + "ti/rc", action) # speed, angle

    # def do_a_simpler_circle(self):
    #     self.check_if_out()
    #     minv = 0.15
    #     maxv = 1.30
                
    #     # sensor_d = ir.ir[0]
    #     if minv <= sensor_d and sensor_d <= maxv:
    #         service.send(service.topicCmd + "ti/rc", f"0.15 {0.40 + 0.05 * (1-(sensor_d-minv)/(maxv-minv))}") # turning        # speed, angle
    #         print("This", 0.40 + 0.05 * (1-(sensor_d-minv)/(maxv-minv)))
    #         pass
    #     else:
    #         service.send(service.topicCmd + "ti/rc", "0.15 0.0") # drive straight # speed, angle
    #         pass
    
    def do_a_const_circle(self):
        self.check_if_out()
        service.send(service.topicCmd + "ti/rc", "0.25 0.68")

    # def do_a_const_circle_i(self): #TODO check_if_out does not account for opposite direction 
    #     self.check_if_out()
    #     service.send(service.topicCmd + "ti/rc", f"0.25 -0.68")
    
    
    ### Drive Out ###
    def check_if_out(self):
        # Use drived distance
        # if pose.tripB >= self.drive_dist:
        
        # Use angle ## Jank due to pose.tripBh after reset 
        # if not self.angle_changed and pose.tripBh < self.saved_angle:
        #     self.angle_changed = True
        # if self.angle_changed and pose.tripBh >= self.saved_angle:
        #     service.send(service.topicCmd + "ti/rc", f"0.0 0.0")
        #     self.job = self.get_out
    
        # Use time
        if self.exit_timer == None:
            self.exit_timer = time()
        # if abs(time() - self.exit_timer) >= 9.0: # old
        if abs(time() - self.exit_timer) >= 6.0:
            service.send(service.topicCmd + "ti/rc", f"0.0 0.0")
            # input("Done")
            self.job = self.get_out
    
    def get_out(self):
        sensor_f = ir.ir[1]
        if self.exit_state == 0: # drive straigt until sensor detect
            action = "0.5 0.0"
            if sensor_f <= 0.2:
                self.exit_state    = 1
                self.set_turn_time = time()

        if self.exit_state == 1: # buffer for robot to align with wall
            action = "0.3 0.0"
            if self.set_turn_time != None and (time() - self.set_turn_time) >= 1.0:
                self.exit_state    = 2
                self.set_turn_time = time()
        
        elif self.exit_state == 2: # turn until robot is parallel with wall
            action = "-0.05 -0.4"
            if self.set_turn_time != None and (time() - self.set_turn_time) >= 3.8:
                self.set_turn_time = None
                self.exit_state    = 3
                
        elif self.exit_state == 3:
            action = "0.15 0.0"
                
            if edge.on_line:
                service.send(service.topicCmd + "ti/rc", f"0.0 0.0")            
                return TaskState.SUCCESS
        service.send(service.topicCmd + "ti/rc", action) # speed, angle
        # print(self.exit_state)

    def loop(self):
        # print("circle state:", self.circle_state, "bh:", pose.tripBh)
        self.job()
        return TaskState.EXECUTING
