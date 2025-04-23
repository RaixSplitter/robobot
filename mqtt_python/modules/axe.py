from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
from uservice import service
import time 

class Axe(Task):
    def __init__(self):
        super().__init__(name='axe')
        self.dont_move = 0.0
        self.drive_fast = 0.75

        self.have_run_through = False
        
        self.obstacle_active = False   
        self.obstacle_start_time = None
        self.obstacle_durations = []   

        self.free_active = False       
        self.free_start_time = None
        self.free_durations = []       

        self.cross_time = 2.0
        self.cross_length = 1.6

        self.is_running = False  
        self.current_trip = 0
        self.check_point_trip = 0
        self.checkpoint_set = False
        self.drive_forward = True

        self.turn_to_face_ball = False
        self.turn_angle = 2

    def loop(self):
        if not self.checkpoint_set:
            self.check_point_trip = pose.tripB
            self.checkpoint_set = True
        if self.drive_forward:
            service.send(service.topicCmd + "ti/rc", "0.1 0.0") # drive straight # speed, angle
            # print(f"{pose.tripB:.2f}, {self.length_to_pose}")
            if pose.tripB-self.check_point_trip >= 0.35:
                service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                self.drive_forward = False
        elif not self.drive_forward:
            distance = ir.ir[1]
            if distance < 0.40:
                if not self.obstacle_active:
                    # We cant go through the axe right now
                    self.obstacle_active = True
                    self.obstacle_start_time = time.time()

                    # When the axe shadows the IR sensor we note the time there was a gap
                    if self.free_active:
                        self.free_active = False
                        duration = time.time() - self.free_start_time
                        self.free_durations.append(duration)
                        print(f"Free interval: {duration:.2f} s")

                if not self.is_running:
                    edge.set_line_control_targets(
                        target_velocity=self.dont_move,
                        target_position=0.0
                    )
                    pose.tripBreset()
                    

            
            else:
                if not self.free_active:
                    self.free_active = True
                    self.free_start_time = time.time()

                    # Once the axe does not shadow the IR sensor we note the time it was free
                    if self.obstacle_active:
                        self.obstacle_active = False
                        duration = time.time() - self.obstacle_start_time
                        self.obstacle_durations.append(duration)
                        print(f"Obstacle interval: {duration:.2f} s")

                
                if not self.is_running and len(self.free_durations) >= 1:
                    self.is_running = True
                    # edge.set_line_control_targets(
                    #     target_velocity=self.drive_fast,
                    #     target_position=0.0
                    # )
                    service.send(service.topicCmd + "ti/rc", f"{self.drive_fast} 0.0")
                    
            if self.is_running:
                if pose.tripB >= self.cross_length:
                    self.have_run_through = True
                    service.send(service.topicCmd + "ti/rc", f"0.0 0.0")

            
            if self.have_run_through:
                # edge.set_line_control_targets(target_velocity=0.0, target_position=0.0)
                pose.tripBreset()
                self.turn_to_face_ball = True
                self.have_run_through = False
            
            if self.turn_to_face_ball:
                # Turn Right? 
                if abs(pose.tripBh) >= self.turn_angle:
                    print("Done turning right")
                    self.has_turned = True
                    service.send(service.topicCmd + "ti/rc", "0.0 0.0") # stop # speed, angle
                    return TaskState.SUCCESS
                else:
                    service.send(service.topicCmd + "ti/rc", "0.0 -0.5") # turn right # speed, angle
                    
        # time.sleep(0.05)
        return TaskState.EXECUTING
