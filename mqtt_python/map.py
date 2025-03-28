### Utilities ###
N,E,S,W = [*"NESW"]
deg     = {xyz:i*90 for i,xyz in enumerate("NESW")}
minmax  = lambda a,b: (min(a,b),max(a,b))

### Connections ###
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

unique_map_speed = {minmax(1,2): 0.35, minmax(2,6): 0.15} # sorted order, use minmax
unique_map_turn  = {(0,1,2): 0.5, (7,10,3): 3, (7,10,9): 3} # unique order

### Main ###
class master_map:
    def __init__(self, path : list, turn : dict, robot_param : dict, init_node = (0, N)):
        ### initiate nodes
        self.nodes        = {n:path_node(n) for n in node_connections.keys()}
        self.current_node = self.nodes[init_node[0]]
        self.current_d    = init_node[1] # from which NESW is it comming from
        self.prev_n       = None
        self.queue        = None
        self.path         = path # nodes the robot has to visit
        self.turn         = turn
        self.robot_param  = robot_param
        self.default_param = {k:v for k,v in robot_param.items()}
        self.unique_map_speed = unique_map_speed
        self.unique_map_turn  = unique_map_turn
        
        ### initiate paths
        for n in self.nodes:
            for np in node_connections[n]:
                nn, *out_in = np
                self.nodes[n].add_path(self.nodes[nn], out_in)
        self.go_to(self.path[0])
    
    def next_action(self):
        # check for end reached
        if len(self.queue) == 1:
            print("Target reached")
            self.queue = None
            self.path = self.path[1:]
            if len(self.path) != 0:
                self.go_to(self.path[0])
                return
            else:
                print("End")
                return
        # check for no initial queue
        elif self.queue == None:
            print("Queue is empty!")
            return
        node, nnext = self.queue[:2]
        # print(node, self.queue, self.current_node.n)
        out, enter = self.current_node.neighbour[self.nodes[nnext]]
        
        action = ""
        ##### out-commented for simpler out
        # if len(out) == 2: 
        #     extra, out = out
        #     action += self.turn[(deg[self.current_d]-deg[extra] + 180)%360] + " - "
        #     self.current_d = extra
        ## simplere out
        out = out[-1]
        
        ### Set action/state/params
        action = self.turn[(deg[self.current_d]-deg[out])%360]
        self.robot_state = action
        self.robot_param["move_speed"]   = self.unique_map_speed.get(minmax(node,nnext),      self.default_param["move_speed"])
        self.robot_param["time_to_turn"] = self.unique_map_turn.get((self.prev_n,node,nnext), self.default_param["time_to_turn"])
        
        print(f"({node}>{nnext})",action)
        
        ### Update to-next-node + queue
        self.prev_n = self.current_node.n
        self.current_node = self.nodes[nnext]
        self.current_d = enter
        self.queue = self.queue[1:]
        
    def go_to(self, nidx):
        ### find path
        target = self.nodes[nidx]
        if self.current_node == target:
            print("Already at target!")
            return
        visited   = {self.current_node}
        bfs_nodes = [(self.current_node,[])]
        ### run bfs
        for node,node_path in bfs_nodes:
            node_path = [*node_path,node.n]
            if target in node.neighbour:
                node_path.append(target.n)
                break
            for nneighbour in node.neighbour:
                if nneighbour in visited:
                    continue
                bfs_nodes.append((nneighbour, node_path))
                visited.add(nneighbour)
        
        ### queue path
        self.queue = node_path # set node queue
        self.next_action()
    
    def change_node(self, node_idx, new_node): # for custom path nodes
        self.nodes[node_idx] = new_node
    
        
class path_node:
    def __init__(self, n):
        self.n = n
        self.neighbour = dict()
    
    def add_path(self, nn, out_in):
        self.neighbour[nn] = out_in 
    

if __name__ == "__main__":
    map = master_map([9,2,6,0,5], {0: "U turn", 90:"Turn right", 180:"Straight", 270:"Turn left"}) # init map

    while map.queue != None:
        input()
        map.next_action() # simulate target reached > new action
