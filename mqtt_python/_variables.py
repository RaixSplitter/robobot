from enum import Enum
from math import pi
from modules.axe import Axe
from modules.eight import Eight
from modules.roundabout_s import Roundabout
from modules.navigate_to_pose import NavigateToPose
from modules.navigate_to_drop_off import NavigateToDropOff
from modules.navigate_to_pose import NavigateToPose, PoseTarget
from modules.luggage import RetrieveLuggage
from modules.ball_detection import pose_est_ball_from_img
from modules.seesaw import Seesaw
from modules.topple_basket import TOPPLE_BASKET
from modules.deliver_golf_ball import DeliverGolfBall

### Utilities ###
deg     = {xyz:i*90 for i,xyz in enumerate("NESW")}
minmax  = lambda a,b: (min(a,b),max(a,b))
N,E,S,W = "NESW"

### STATE ###
class State(Enum):
	START = 0
	PHOTO_MODE = 1
	FOLLOW_LINE = 2
	U_TURN = 3
	LOST = 4
	TURN_LEFT = 5
	TURN_RIGHT = 6
	TRY_RECOVER = 7
	SOLVING_TASK = 8
	END_PROGRAM = 1000
	TESTING = 100000

### TASK ###
class Task:
	AXE         		= Axe()
	TOPPLE_BASKET       = TOPPLE_BASKET()
	EIGHT       		= Eight()
	ROUNDABOUT  		= Roundabout()
	SEESAW     			= Seesaw()
	DELIVER_GOLF_BALL 	= DeliverGolfBall()
	RETRIEVE_LUGGAGE  	= RetrieveLuggage()
	GET_LUGGAGE       	= NavigateToPose(target = PoseTarget.ARUCO_LD)
	DELIVER_LUGGAGE 	= NavigateToDropOff(target = PoseTarget.D)
	GET_BALL		   	= NavigateToPose(target = PoseTarget.BLUE_BALL)
	DELIVER_BALL 		= NavigateToDropOff(target = PoseTarget.C)
	EXIT                = NavigateToPose(target = PoseTarget.EXIT)
	

### ROBOT VALUES ###
# Default params, can and will be overwritten in `map.py`
# speed        | 0.1 | 0.2 | 0.3
# 180 deg turn | 4.5 | 4.0 | 4.2
default_params = {
	"turn_angle": 1.5, 			# how long does a normal left or right turn take in seconds
	"turn_speed": 0,				# should it drive as it turns | 0 = no driving
	"move_speed": 0.25, 			# max 1
	"skip_cross": 0,				# number of crossroads that are skipped (E1-4, E6-7)
	"pid_values": (1.8, 0.0, 0.6), 	# p, i, d
	"task_list": [],
}

### Connections/uniques ###
node_connections = { # n : ((np, from, to),...)
	0: ((1,S,E),),
	1: ((0,E,S),(2,E+N,N),(4,E+S,N)),
	2: ((1,N,W),(3,S,N),(6,E,W)),
	3: ((2,N,S),(10,S,W)), # going downstairs: s(7,N+E,W)
	4: ((1,N,E),(5,S,N),(8,E,W)),
	5: ((4,N,S),(6,S,N),(100,E,W)),
	6: ((5,N,S),(7,S,N)),
	7: ((6,N,S),(10,S,N)), # < add 3 for going upstairs
	8: (),
	9: ((10,S,E),),
   10: ((3,S+W,S),(7,S+N,S),(9,S+E,S)),
#    100:((4,S,N),) # task 8
   100:((5,W,E),) # task 8
}

class uniques:
	map_speed = { # consistent order, use minmax
    	minmax(1,2): 0.25, 
    	# minmax(1,2): 0.20, # Temp for test 
		minmax(1,4): 0.15,
		minmax(4,5): 0.15,
		minmax(2,6): 0.15,
	}
	map_turn = { 
		(0,1,4): pi/5, 
		(0,1,2): pi/4, 
		# (2,3,10): pi/4,
		(7,10,3): pi*1/2, # 3/4 too much 
		(7,10,9): pi*1/2, # 3/4 too much 
	}
	map_turn_speed = {
		minmax(1,2): 0.2,
		minmax(1,4): 0.2,
	}
	limits = {
		# minmax(1,2)   : 1,
		minmax(2,6)   : 1,
		minmax(5,100) : 1,
	}
	cross_skip = { # consistent order, use minmax
		minmax(1,4) : 1,
		minmax(6,7) : 1,
	}
	pid_values = { # consistent order, use minmax
		# minmax(4,5) : (1.5, 0.0, 0.8)
	}
	delegate_task = {
		# (0,1): [Task.EIGHT, Task.ROUNDABOUT],
		# (0,1): [Task.ROUNDABOUT],
		# (0,1): [Task.TOPPLE_BASKET],

		(4,8): [Task.AXE, Task.GET_BALL, Task.DELIVER_BALL, Task.EXIT],
		# (5,100): [Task.EIGHT, Task.ROUNDABOUT],
		(5,4): [Task.TOPPLE_BASKET],
		# (5,100): [Task.TOPPLE_BASKET],
		(2,6): [Task.SEESAW,],
		(10,3): [Task.DELIVER_GOLF_BALL,],
	}