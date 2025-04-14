from enum import Enum
from modules.axe import Axe
from modules.eight import Eight
from modules.roundabout import Roundabout
from modules.navigate_to_pose import NavigateToPose
from modules.ball_detection import pose_est_ball_from_img

### Utilities ###
deg     = {xyz:i*90 for i,xyz in enumerate("NESW")}
minmax  = lambda a,b: (min(a,b),max(a,b))
N,E,S,W = "NESW"

### STATE ###
class State(Enum):
	START = 0
	PHOTO_MODE = 1
	FOLLOW_LINE = 2
	LOST = 4
	TURN_LEFT = 5
	TURN_RIGHT = 6
	TRY_RECOVER = 7
	SOLVING_TASK = 8
	END_PROGRAM = 1000
	TESTING = 100000

### TASK ###
class Task(Enum):
	AXE = 0
	EIGHT = 1
	ROUNDABOUT = 2
	NAVIGATE = 3

tasks = {
	Task.AXE : Axe(),
	Task.EIGHT : Eight(),
	Task.ROUNDABOUT : Roundabout(),
	Task.NAVIGATE : NavigateToPose()
}

### ROBOT VALUES ###
# Default params, can and will be overwritten in `map.py`
# speed        | 0.1 | 0.2 | 0.3
# 180 deg turn | 4.5 | 4.0 | 4.2
default_params = {
	"time_to_turn": 2, 				# how long does a normal left or right turn take in seconds
	"move_speed": 0.25, 			# max 1
	"skip_cross": 0,				# number of crossroads that are skipped (E1-4, E6-7)
	"pid_values": (0.8, 0.0, 0.1), 	# p, i, d
	"current_task": None,
}

### Connections/uniques ###
node_connections = { # n : ((np, from, to),...)
    0: ((1,S,E),),
    1: ((0,E,S),(2,E+N,N),(4,E+S,N)),
    2: ((1,N,W),(3,S,N),(6,E,W)),
    3: ((2,N,S),(10,S,N)), # going downstairs: s(7,N+E,W)
    4: ((1,N,E),(5,S,N),(8,E,W)),
    5: ((4,N,S),(6,S,N)),
    6: ((5,N,S),(7,S,N)),
    7: ((6,N,S),(10,S,N)), # < add 3 for going upstairs
    8: (),
    9: ((10,S,N),),
   10: ((3,S+W,S),(7,S+N,S),(9,S+E,S)),
}

uniques = {
	"map_speed": { # consistent order, use minmax
		minmax(1,2): 0.25, 
		minmax(2,6): 0.15
	},
	"map_turn": { 
		(0,1,2): 0.2, 
		(7,10,3): 3, 
		(7,10,9): 3
	},
	"skipping_cross": { # consistent order, use minmax
		minmax(1,4) : 2,
		minmax(6,7) : 2,
	},
	"pid_values": { # consistent order, use minmax
		minmax(1, 2) : (2.0, 0.0, 0.4)
	},
	"delegate_task": { 
		(4,8): [Task.AXE],
	}
}