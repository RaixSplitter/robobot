from enum import Enum

class TaskState(Enum):
    EXECUTING = 0
    FAILURE = 1
    LOST = 2
    SUCCESS = 3
    SUCCESS_SKIP = 4

class Task():

    def __init__(self, name: str):
        self.name = name
        self.state = TaskState.EXECUTING

    def loop(self) -> TaskState:
        raise NotImplementedError("Please Implement a loop method")

    def has_succeeded(self):
        return TaskState.SUCCESS
    
    def is_lost(self):
        return TaskState.LOST
    
    def has_failed(self):
        return TaskState.FAILURE
    
    