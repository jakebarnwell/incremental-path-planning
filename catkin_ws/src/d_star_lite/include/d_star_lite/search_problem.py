from grid import *
from copy import deepcopy

class IncrementalSearchProblem(object):
    # For now we have a dict mapping robot (x,y) --> Graph, i.e. its understanding
    #  of the world when it's sitting at location (x,y)
    def __init__(self, world, start_node=None, goal_node=None):
        self._start_node = start_node
        self._goal_node = goal_node
        self._world = world

    def __repr__(self):
        return "{}(start_node={}, goal_node={}, world={})".format(type(self).__name__, repr(self._start_node), repr(self._goal_node), repr(self._world))

    __str__ = __repr__

    def __eq__(self, other):
        return self._start_node == other._start_node \
            and self._goal_node == other._goal_node \
            and self._world == other._world

    def copy(self):
        return deepcopy(self)

    def get_graph(self):
        return self._world.belief

    def draw_state(self, grid, start=None, goal=None, robot_location=None, path=None):
        axes = grid.draw()
        if robot_location:
            grid.draw_cell_circle(axes, robot_location, color='red')

    def update_world(self, intended_path):
        return self._world.update_world(intended_path)

    def is_robot_at_goal(self):
        return self._world._is_robot_at_goal

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

