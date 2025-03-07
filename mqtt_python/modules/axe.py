from mqtt_python.modules.task import Task

class Axe(Task):

    def __init__(self):
        super().__init__(name = 'axe')

    def loop(self):
        pass