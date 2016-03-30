from grid import *

TOLERANCE = 0.01

class World: #todo add __str__/__repr__ #todo move into world.py
#todo add is_at_goal flag
#todo assumptions: world can change, but give up if currently can't reach goal
#obstacles may appear/disappear/move, but they will never crush the robot
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
    def __init__(self, init_grid_ground_truth, robot_start_position, vision_radius=3):
        # The radius the robot can "see" for its belief state
        self._VISION_RADIUS = vision_radius

        # The current time step, robot's position, and path travelled so far
        self._time = 0
        self._robot_position = robot_start_position
        self._path_travelled = []

        # Stores what the world really looks like currently
        self._grid_ground_truth = init_grid_ground_truth

        # Stores what the robot "sees" or "believes" to be the world currently
        self._grid_belief_state = init_grid_ground_truth.clone_template()
        self._update_belief_state()


    def update_world(self, next_robot_position):
        """
        Updates the ground truth grid AND the belief state grid,
        and returns a GRAPH-version of the belief state.
        """
        self._time += 1
        self._path_travelled.append(next_robot_position)

        # Update ground truth, e.g. if there are moving obstacles. Note that
        #  we must update the robot's position in the grid as well.
        rx, ry = self._robot_position
        nrx, nry = next_robot_position
        self._grid_ground_truth.mark_cell_as(nrx, nry, CELL_ROBOT)
        self._grid_ground_truth.mark_cell_as(rx, ry, CELL_FREE)

        # Store new robot position. Must happen before _update_belief_state
        #  is called.
        self._robot_position = next_robot_position

        # Re-calculate belief state. Must happen after the new _robot_position
        #  is set.
        self._update_belief_state()

        # Graph-ify the belief state and return it
        return self.belief

    def _update_belief_state(self):
        """
        Updates the robot's belief state of the world. This
        information typically depends on the robot's position,
        e.g. what the robot can "see" from where it's at.
        """
        ground_truth_grid_array = self._grid_ground_truth.get_grid_array()
        num_cols, num_rows = self._grid_ground_truth.size
        rx, ry = self._grid_ground_truth.cell_center(*self._robot_position)
        for col in xrange(num_cols):
            for row in xrange(num_rows):
                x, y = self._grid_ground_truth.cell_center(col, row)
                if (rx - x)**2 + (ry - y)**2 <= (self._VISION_RADIUS * (1 + TOLERANCE))**2:
                    self._grid_belief_state.set_cell(col, row, self._grid_ground_truth.get_cell(col,row))
                    # print "Visible {} --> {}: {}".format(self._robot_position, (col, row), self._grid_ground_truth.get_cell(col,row))
                else:
                    pass
                    # Instead of overwriting e.g. "0"s (free cell) to every cell that
                    #  the robot can't see, just do nothing to them, so it retains its
                    #  "knowledge" of what used to be at the cell, even after it is
                    #  no longer "nearby" that cell. Hence, if that cell has CHANGED,
                    #  the robot doesn't know that.
        return True

    @property
    def robot_position(self):
        return self._robot_position

    @property
    def time(self):
        return self._time

    @property
    def path_travelled(self):
        return self._path_travelled

    @property
    def belief(self):
        """
        D* Lite should call this. It returns the current belief state as a graph.
        """
        return self._grid_belief_state.to_graph()

    @property
    def belief_grid(self):
        """
        Returns current belief state as a grid. Helpful if you want to draw
        pretty grids.
        """
        return self._grid_belief_state
