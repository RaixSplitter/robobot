from __future__ import annotations
from _variables import N,E,S,W, deg, minmax, node_connections, uniques, default_params

### Main ###
class master_map:
    def __init__(self, path : list, turn : dict, robot_param : dict, init_node = (0, N), n_skip = 0):
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
        if n_skip > 0:
            self.skip(n_skips=n_skip)
    
    def next_action(self):
        # check for end reached
        print(self.queue)
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
        direction = minmax(current_n, next_n)
        self.robot_param["move_speed"] = self.uniques.map_speed.get(direction,                 self.default_param["move_speed"])
        self.robot_param["turn_angle"] = self.uniques.map_turn.get((prev_n,current_n, next_n), self.default_param["turn_angle"])
        self.robot_param["turn_speed"] = self.uniques.map_turn_speed.get(direction,            self.default_param["turn_speed"])
        self.robot_param["skip_cross"] = self.uniques.cross_skip.get(direction,                self.default_param["skip_cross"])
        self.robot_param["pid_values"] = self.uniques.pid_values.get(direction,                self.default_param["pid_values"])
        self.robot_param["task_list" ] = self.uniques.delegate_task.get((current_n,next_n),    self.default_param["task_list" ])
        if direction in self.uniques.limits and self.uniques.limits[direction] != 0:
            self.uniques.limits[direction] -= 1
            if self.uniques.limits[direction] != 0:
                return
            self.nodes[current_n].remove_path(self.nodes[next_n])
            self.nodes[next_n].remove_path(self.nodes[current_n])
        
    def change_node(self, node_idx, new_node): # for custom path nodes
        self.nodes[node_idx] = new_node
    
    def skip(self, n_skips : int):
        print("First node", self.current_node.n)
        for _ in range(n_skips):
            self.next_action()
            print(self.current_node.n)
        print("C node", self.current_node.n)
    
    

class path_node:
    def __init__(self, n : int):      
        self.n = n
        self.neighbour = dict()
    
    def add_path(self, nn : path_node, out_in : str):
        self.neighbour[nn] = out_in 
    
    def remove_path(self, nn : path_node):
        if nn in self.neighbour:
            del self.neighbour[nn]
    

if __name__ == "__main__":
    map = master_map([2,6,2,6], {0: "U turn", 90:"Turn right", 180:"Straight", 270:"Turn left"}, default_params) # init map

    while map.queue != None:
        input()
        map.next_action() # simulate target reached > new action
