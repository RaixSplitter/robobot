from _variables import Task, N,E,S,W, deg, minmax, node_connections, uniques


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
        self.robot_param  = robot_param # from mqtt-client, shared references
        self.default_param = {k:v for k,v in robot_param.items()} # copy, no references
        self.uniques       = uniques
        
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
        node, next_n = self.queue[:2]
        # print(node, self.queue, self.current_node.n)
        out, enter = self.current_node.neighbour[self.nodes[next_n]]
        
        action = ""
        ##### out-commented for simpler out # if len(out) == 2:  #     extra, out = out #     action += self.turn[(deg[self.current_d]-deg[extra] + 180)%360] + " - " #     self.current_d = extra ## simplere out
        out = out[-1]
        
        ### Update state
        action = self.turn[(deg[self.current_d]-deg[out])%360]
        self.robot_state = action
        self.update(self.prev_n, node, next_n)
        
        print(f"({node}>{next_n})",action)
        
        ### Update to-next-node + queue
        self.prev_n = self.current_node.n
        self.current_node = self.nodes[next_n]
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
    
    def update(self, prev_n, current_n, next_n):
        self.robot_param["move_speed"  ] = self.uniques["map_speed"     ].get(minmax(current_n, next_n), self.default_param["move_speed"  ])
        self.robot_param["time_to_turn"] = self.uniques["map_turn"      ].get((prev_n,current_n),        self.default_param["time_to_turn"])
        self.robot_param["skip_cross"  ] = self.uniques["skipping_cross"].get(minmax(current_n,next_n),  self.default_param["skip_cross"  ])
        self.robot_param["pid_values"  ] = self.uniques["pid_values"    ].get(minmax(current_n,next_n),  self.default_param["pid_values"  ])
        self.robot_param["current_task"] = self.uniques["delegate_task" ].get((current_n,next_n),        self.default_param["current_task"])
        
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
