class IncrementalSearchProblem(object):
    # For now we have a dict mapping robot (x,y) --> Graph, i.e. its understanding
    #  of the world when it's sitting at location (x,y)
    def __init__(self, world, start_node=None, goal_node=None):
        self._start_node = start_node
        self._goal_node = goal_node
        self._world = world
        
    def __repr__(self):
        return "{}(start_node={}, goal_node={}, world={})".format(type(self).__name__, repr(self._start_node), repr(self._goal_node), repr(self._world))
    
    def __str__(self):
        repr_world = self._world if len(self._world) == 0 else "{...}"
        return "{}(start_node={}, goal_node={}, world={})".format(type(self).__name__, repr(self._start_node), repr(self._goal_node), repr_world)
    
    def get_graph(self, robot_location):
        return self._world[robot_location]
    
    def draw_state(self, grid, start=None, goal=None, robot_location=None, path=None):
        axes = grid.draw()
        if robot_location:
            grid.draw_cell_circle(axes, robot_location, color='red')
                
    @property
    def start_node(self):
        return self._start_node
    
    @property
    def goal_node(self):
        return self._goal_node
    
    @start_node.setter
    def start_node(self, new_start_node):
        self._start_node = new_start_node
        
    @goal_node.setter
    def goal_node(self, new_goal_node):
        self._goal_node = new_goal_node


# Work in progress... need to do the right stuff with grids and make them
#  mutable I think
class World:
    """
    A World object represents the belief-state of a robot. In 
    particular, a World object stores what the world (as a grid)
    "looks like" to the robot at any particular point in time, as
    well as storing what the world "really" looks like.
    
    A World object also stores the current position of the robot as
    a tuple (x,y), as well as the current "time" (time starts at 0 and
    is incremented by 1 whenever update_world is called), and the
    path travelled by the robot thus far in this World.
    
    The only way to mutate anything in this class is via the update_world
    method.
    """
    def __init__(self, init_grid_ground_truth, robot_start_position, vision_radius=5):
        # Stores what the world really looks like currently
        self._grid_ground_truth = init_grid_ground_truth
        # Stores what the robot "sees" or "believes" to be the world currently
        self._grid_belief_state = None
        self._update_belief_state()
        
        # The current time step, robot's position, and path travelled so far
        self._time = 0
        self._robot_position = robot_start_position
        self._path_travelled = []
        
        # The radius the robot can "see" for its belief state
        self._VISION_RADIUS = vision_radius
        
    def update_world(self, next_robot_position):
        """
        Updates the ground truth grid AND the belief state grid,
        and returns a GRAPH-version of the belief state.
        """
        _time += 1
        _path_travelled.append(next_robot_position)
        
        # Update ground truth
        pass
    
        # Re-calculate belief state
        self._update_belief_state()
        
        # Graph-ify the belief state and return it
        return self._grid_belief_state.to_graph()
        
    def _update_belief_state(self):
        """
        Updates the robot's belief state of the world. This
        information typically depends on the robot's position,
        e.g. what the robot can "see" from where it's at.
        """
        grid_array = self._grid_ground_truth.grid_array
        size = self._grid_ground_truth.size
        print size
        x, y = self._robot_position
        cx, cy = self._grid_ground_truth.cell_xy(x, y)
        xmax, ymax = size
        for i in xrange(xmax):
            for j in xrange(ymax):
                ix, iy = self._grid_ground_truth.cell_xy(i, j)
                if (cx - ix)**2 + (cy - jy)**2 <= self._VISION_RADIUS**2:
                    self._grid_belief_state[i, j] = grid_array
                else:
                    self._grid_belief_state[i, j] = 0 # It thinks it's clear
        return True
            
        
    @property
    def robot_position(self):
        return _robot_position
    
    @property
    def time(self):
        return _time
    
    @property
    def path_travelled(self):
        return _path_travelled

# class World:
#     __init__(grid)
#     # --> graph
#     robot_xy ;; current tuple (x,y) of robot position
#     path_travelled ;; path the robot has travelled [(x,y),...] updated in backgroudn

# d*lite has to call update_world(robot_position_node);; returns the new graph
#  ground_truth_graph
#  visible_world_graph

# (3,4)

























