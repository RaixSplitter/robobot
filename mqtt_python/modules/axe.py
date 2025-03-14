from modules.task import Task, TaskState
from sir import ir
from sedge import edge
from spose import pose
import time 

class Axe(Task):
    def __init__(self):
        super().__init__(name='axe')
        self.dont_move = 0.0
        self.drive_fast = 0.6

        self.have_run_through = False
        
        self.obstacle_active = False   
        self.obstacle_start_time = None
        self.obstacle_durations = []   

        self.free_active = False       
        self.free_start_time = None
        self.free_durations = []       

        self.cross_time = 2.0
        self.cross_length = 50

        self.is_running = False  

    def loop(self):
        while self.state == TaskState.EXECUTING:
            distance = ir[0]  
            print(f"IR distance: {distance} cm")  

            if 10 < distance < 80:
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

           
            elif distance > 100:
                if not self.free_active:
                    self.free_active = True
                    self.free_start_time = time.time()

                    # Once the axe does not shadow the IR sensor we note the time it was free
                    if self.obstacle_active:
                        self.obstacle_active = False
                        duration = time.time() - self.obstacle_start_time
                        self.obstacle_durations.append(duration)
                        print(f"Obstacle interval: {duration:.2f} s")

              
                if not self.is_running and len(self.free_durations) > 1:
                    self.is_running = True
                    edge.set_line_control_targets(
                        target_velocity=self.drive_fast,
                        target_position=0.0
                    )
                    
            if self.is_running:
                if pose.tripB >= self.cross_length:
                    self.have_run_through = True

            
            if self.have_run_through:
                edge.set_line_control_targets(target_velocity=0.0, target_position=0.0)
                self.state = TaskState.SUCCESS

            time.sleep(0.05)
