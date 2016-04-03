from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Ellipse
from graph import Graph
import time as time_library
from IPython import display


class WrongGridFormat(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return "Wrong grid format. Use 0 for free space, 1 for obstacle, S and G for start and goal, respectively, and R for robot."

class WrongCellType(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Invalid cell type. A cell type should be one of: " + repr(VALID_CELL_TYPES)

PREFERRED_MAX_FIG_WIDTH = 12
PREFERRED_MAX_FIG_HEIGHT = 8

# Easily refer to cell-types
CELL_FREE = 0
CELL_OBSTACLE = 1
CELL_START = 7
CELL_GOAL = 9
VALID_CELL_TYPES = [CELL_FREE, CELL_OBSTACLE, CELL_START, CELL_GOAL]

COLOR = { \
    "free": "white", \
    "obstacle": "#333333", \
    "new-obstacle": "None", \
    "robot": "black", \
    "start": "#00DD44", \
    "goal": "red", \
    "path-travelled": "green", \
    "path-future": "#DD0000"
    }

class Grid(object):
    def __init__(self, num_cols=10, num_rows=10, figsize=None):
        self.generate_grid(num_cols, num_rows)

        self.xlimits = (minx, maxx) = (0, num_cols)
        self.ylimits = (miny, maxy) = (0, num_rows)
        self.cell_size = (maxx-minx) / num_cols, (maxy-miny) / num_rows

        if figsize:
            self.figsize = figsize
        else:
            width, height = maxx - minx, maxy - miny
            if width > height:
                self.figsize = (PREFERRED_MAX_FIG_WIDTH, height * PREFERRED_MAX_FIG_WIDTH / width)
            else:
                self.figsize = (width * PREFERRED_MAX_FIG_HEIGHT / height, PREFERRED_MAX_FIG_HEIGHT)

    def __eq__(self, other):
        return self.xlimits == other.xlimits and self.ylimits == other.ylimits \
            and self.grid_array == other.grid_array

    @classmethod
    def create_from_file(grid_class, grid_file, figsize=None):
        gfile = open(grid_file)
        grid = grid_class.create_from_str(gfile.read(), figsize=figsize)
        gfile.close()
        return grid

    @classmethod
    def create_from_str(grid_class, grid_str, figsize=None):
        # Don't ask.
        def string2value(s):
            string2valueDict = {"0":0, "1":1, "S":7, "G":9}
            return string2valueDict[s]

        lines = map(str.split, filter(lambda s:not s.startswith('#') and len(s)>0, map(str.strip,grid_str.split('\n'))))
        num_rows = len(lines)
        num_cols = len(lines[0])

        for line in lines:
            if not num_cols == len(line):
                raise WrongGridFormat
        grid_array = np.zeros([num_cols,num_rows])
        for row in range(num_rows):
            for col in range(num_cols):
                value = lines[row][col]
                if value not in ["0","1","S","G","R"]:
                    raise WrongGridFormat
                grid_array[col,num_rows-1 - row] = string2value(value)

        grid = grid_class(num_cols, num_rows, figsize)
        grid.grid_array = grid_array

        return grid

    def clone_template(self):
        num_cols, num_rows = self.size
        new_grid = Grid(num_cols, num_rows, self.figsize)
        return new_grid

    @property
    def size(self):
        return self.grid_array.shape

    def get_grid_array(self):
        return self.grid_array

    def get_cell(self, x, y):
        return self.grid_array[x, y]

    def set_cell(self, x, y, val):
        self.grid_array[x, y] = val
        return True

    def generate_grid(self, num_cols, num_rows):
        self.grid_array = np.zeros([num_cols, num_rows])

    def add_random_obstacles(self, num_obs):
        """Clear grid and add random obstacles"""

        # Only mess with cells that are completely free (i.e. a non-blocked cell
        #  that is not a robot, start, or goal cell)
        free_idx = zip(*np.where(self.grid_array == 0))
        num_free = len(free_idx)
        num_obs = min(num_free, num_obs)
        for i in range(num_obs):
            obs_idx = np.random.randint(0, num_free-1)
            self.grid_array[free_idx.pop(obs_idx)] = 1
            num_free = num_free - 1

    def mark_cell_as(self, x, y, what_type):
        if what_type not in VALID_CELL_TYPES:
            raise WrongCellType

        self.grid_array[x, y] = what_type

    def get_cells_of_type(self, what_type):
        if what_type not in VALID_CELL_TYPES:
            raise WrongCellType

        return zip(*np.where(self.grid_array == what_type))

    def clear(self):
        self.grid_array = np.zeros([self.num_cols, self.num_rows])

    def cell_center(self, ix, iy):
        """Returns the center xy point of the cell."""
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits
        cwidth, cheight = self.cell_size
        return minx + (ix+0.5) * cwidth, miny + (iy+0.5) * cheight

    def _cell_vertices(self, ix, iy):
        cwidth, cheight = self.cell_size
        x, y = self.cell_center(ix, iy)
        verts = [(x + ofx*0.5*cwidth, y + ofy*0.5*cheight) for ofx, ofy in [(-1,-1),(-1,1),(1,1),(1,-1)]]
        return verts

    def export_to_dict(self):
        export_dict = {}
        export_dict['grid'] = self.grid_array.tolist()
        return export_dict

    def load_from_dict(self, grid_dict):
        self.grid_array = np.array(grid_dict['grid'])

    def get_goal(self):
        return self.get_cells_of_type(CELL_GOAL)[0]

    # DRAWING METHODS

    def draw(self):
        cols, rows = self.size
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits

        cwidth, cheight = self.cell_size

        x = map(lambda i: minx + cwidth*i, range(cols+1))
        y = map(lambda i: miny + cheight*i, range(rows+1))

        f = plt.figure(figsize=self.figsize)

        hlines = np.column_stack(np.broadcast_arrays(x[0], y, x[-1], y))
        vlines = np.column_stack(np.broadcast_arrays(x, y[0], x, y[-1]))
        lines = np.concatenate([hlines, vlines]).reshape(-1, 2, 2)
        line_collection = LineCollection(lines, color="black", linewidths=0.5)
        ax = plt.gca()
        ax.add_collection(line_collection)
        ax.set_xlim(x[0]-1, x[-1]+1)
        ax.set_ylim(y[0]-1, y[-1]+1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('off')
        self._draw_obstacles(plt.gca())
        self._draw_start_goal(plt.gca())

        return plt.gca()

    def _draw_obstacles(self, axes):
        verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_OBSTACLE)]
        collection_recs = PolyCollection(verts, facecolors=COLOR["obstacle"])
        axes.add_collection(collection_recs)

    def _draw_start_goal(self, axes):
        start_verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_START)]
        goal_verts = [self._cell_vertices(ix, iy) for ix,iy in self.get_cells_of_type(CELL_GOAL)]
        collection_recs = PolyCollection(start_verts, facecolors=COLOR["start"])
        axes.add_collection(collection_recs)
        collection_recs = PolyCollection(goal_verts, facecolors=COLOR["goal"])
        axes.add_collection(collection_recs)

    def draw_cell_circle(self, axes, xy, size=0.6, **kwargs):
        ix, iy = xy
        x, y = self.cell_center(ix, iy)
        xr, yr = size * self.cell_size[0], size * self.cell_size[1]
        axes.add_patch(Ellipse((x,y), xr, yr, **kwargs))

    def draw_path(self, axes, path, *args, **kwargs):
        # Path doesn't really make sense for 1 or 0 vertices
        if len(path) > 1:
            xy_coords = map(lambda idx: self.cell_center(*idx), path)
            xx, yy = zip(*xy_coords)
            axes.plot(xx, yy, *args, **kwargs)

    def to_graph(self):
        xmin, xmax = self.xlimits
        ymin, ymax = self.ylimits

        def neighbors(xy):
            x,y = xy
            deltaX = [i for i in range(-1,2,1) if x+i >= xmin and x+i < xmax]
            deltaY = [j for j in range(-1,2,1) if y+j >= ymin and y+j < ymax]

            n = []
            for dx in deltaX:
                for dy in deltaY:
                    if not(dx == 0 and dy == 0): # gets rid of the original (x,y) case
                        n.append((x + dx, y + dy))
            return n

        graph = Graph()
        node_positions = {}
        # Add all tiles as nodes
        for col in range(xmin, xmax):
            for row in range(ymin, ymax):
                graph.add_node((row,col))
                # Sets the position for visualization:
                node_positions[(row,col)] = (row,col)

        graph.set_node_positions(node_positions)

        # Add bi-directional edges between the 8 cardinal+ordinal neighbors
        obstacles = self.get_cells_of_type(CELL_OBSTACLE)
        for col in range(xmin, xmax):
            for row in range(ymin, ymax):
                # For obstacle blocks, add a infinite-weight edges.
                if (col, row) in obstacles:
                    for neighbor in neighbors((col,row)):
                        graph.add_edge((col,row), neighbor, weight=np.inf, bidirectional=False)
                else:
                    for neighbor in neighbors((col,row)):
                        weight = np.inf if neighbor in obstacles else 1.0
                        graph.add_edge((col,row), neighbor, weight=weight, bidirectional=False)

        return graph
