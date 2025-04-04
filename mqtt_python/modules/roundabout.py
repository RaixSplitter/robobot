from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class Roundabout(Task):
    def __init__(self):
        super().__init__(name='roundabout')
        self.topicCmd = "robobot/cmd/" # send to Teensy T0, T1, or teensy_interface
        self.topicRc = service.topicCmd + "ti/rc"

        self.turn_start = None
        self.turn_rate = 0.6 # turn left
        self.turn_speed = 0.2
        self.move_speed = 0.35 # max 1

        self.Kp = 3 # was 0.5
        self.Ki = 0
        self.Kd = 15 # was 0.4

        self.front_wall_detected = False
        self.has_turned_left = False
        self.target_dist_from_wall = .075
        self.current_dist_from_wall = None

        self.get_to_ramp = False
        self.turn_left = False
        self.has_turned_left_ramp = False
        self.target_h = 90
        self.target_h2 = 30
        self.h_reset = False
        self.tripB_reset = False
        self.circling = False

    def pid_loop(self, dt: float, target: float, current: float) -> float:
        # Avoid extremely small dt
        if dt < 1e-6:
            dt = 1e-6

        error = target - current
        self.cumulative_error += error * dt
        # Potentially clamp integrator more tightly or differently
        self.cumulative_error = max(min(self.cumulative_error, 1.0), -1.0)

        diff_error = (error - self.previous_error) / dt
        self.previous_error = error

        u = (self.Kp * error) \
            + (self.Ki * self.cumulative_error) \
            + (self.Kd * diff_error)
        return max(min(u, 1.0), -1.0)


    def follow_wall(self):
        """ Function called in loop to give follow line commands """
        from uservice import service

        dt = ir.irInterval / 1000.0 # seconds

        u = self.pid_loop(dt, self.target_dist_from_wall, self.current_dist_from_wall)

        command = f"{self.move_speed:.3f} {u:.3f} {time.time()}"
        service.send(self.topicRc, command) # send new turn command, maintaining velocity

    def loop(self):
        self.current_dist_from_wall = ir.ir[1]

        if not self.front_wall_detected and not self.has_turned_left:
            self.follow_wall()
            if ir.ir[1] < .20:
                self.front_wall_detected = True

        if self.front_wall_detected and not self.has_turned_left:
            edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
            if not self.h_reset:
                pose.tripBreset()
                self.h_reset = True
            service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
            if pose.tripBh >= self.target_h:
                self.has_turned_left = True

        if self.front_wall_detected and self.has_turned_left:
            self.follow_wall()
            if pose.tripB > 1.2:
                self.get_to_ramp = True
                self.back_up = True

        if self.get_to_ramp:
            ### Back up a bit
            if self.back_up:
                if not self.tripB_reset:
                    self.tripB_reset = True
                    pose.tripBreset()
                edge.set_line_control_targets(target_velocity = -0.5*self.move_speed, target_position = 0.0)
                if pose.tripB >= .20:
                    self.back_up = False
                    self.tripB_reset = False
                    self.turn_left = True
                    self.h_reset = False
        
            ### Turn left a bit
            if self.turn_left:
                edge.set_line_control_targets(target_velocity = 0.0, target_position = 0.0)
                if not self.h_reset:
                    pose.tripBreset()
                    self.h_reset = True
                service.send(service.topicCmd + "ti/rc", "0.0 0.8") # turn left # speed, angle
                if pose.tripBh >= self.target_h2:
                    self.has_turned_left_ramp = True

            ### Drive onto the ramp
            if self.has_turned_left_ramp:
                service.send(service.topicCmd + "ti/rc", "0.2 0.0") # turn left # speed, angle
                if not self.tripB_reset:
                    self.tripB_reset = True
                    pose.tripBreset()


        if self.turn_start:
            self.turn_start = time.time()
            pose.tripBreset()
        
        if 3 < pose.tripB:
            print("Time turning: ", time.time() - self.turn_start)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")    
            return TaskState.SUCCESS


        return TaskState.EXECUTING
