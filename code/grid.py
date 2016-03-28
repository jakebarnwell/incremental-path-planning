from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Ellipse
from graph import Graph


class WrongGridFormat(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return "Wrong grid format. Use 0 for free space, 1 for obstacle, S and G for start and goal, respectively, and R for robot."
    
PREFERRED_MAX_FIG_WIDTH = 12
PREFERRED_MAX_FIG_HEIGHT = 8
    
class Grid(object):
    def __init__(self, num_cols=10, num_rows=10, xy_limits=None, figsize=None):
        self.generate_grid(num_cols, num_rows)

        if not xy_limits:
            xy_limits = (0, num_cols), (0, num_rows)
        self.set_xy_limits(*xy_limits)
        if not figsize:
            figsize = self.calc_auto_figsize(xy_limits)
        self.figsize = figsize
        
    @property
    def size(self):
        return self.grid_array.shape

    def generate_grid(self, num_cols, num_rows):
        self.grid_array = np.zeros([num_cols, num_rows])

    def set_xy_limits(self, xlimits, ylimits):
        num_cols, num_rows = self.size
        if not isinstance(xlimits,tuple) or not len(xlimits)==2 \
           or not isinstance(ylimits,tuple) or not len(ylimits)==2 \
           or not xlimits[0] < xlimits[1] or not ylimits[0] < ylimits[1]:
            raise ValueError('Specified xlimits or ylimits are not valid.')
        self.xlimits, self.ylimits = xlimits, ylimits
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits
        self.cell_dimensions = (maxx-minx) / num_cols, (maxy-miny) / num_rows

    def calc_auto_figsize(self, xy_limits):
        (minx, maxx), (miny, maxy) = xy_limits
        width, height = maxx - minx, maxy - miny
        if width > height:
            figsize = (PREFERRED_MAX_FIG_WIDTH, height * PREFERRED_MAX_FIG_WIDTH / width)
        else:
            figsize = (width * PREFERRED_MAX_FIG_HEIGHT / height, PREFERRED_MAX_FIG_HEIGHT)
        return figsize

    @classmethod
    def create_from_file(grid_class, grid_file, xy_limits=None, figsize=None):
        gfile = open(grid_file)
        grid = grid_class.create_from_str(gfile.read(), xy_limits=xy_limits, figsize=figsize)
        gfile.close()
        return grid
    @classmethod
    def create_from_str(grid_class, grid_str, xy_limits=None, figsize=None):
        # Don't ask.
        def string2value(s):
            string2valueDict = {"0":0, "1":1, "S":7, "G":9, "R":8}
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

        grid = grid_class(num_cols, num_rows, xy_limits, figsize)
        grid.grid_array = grid_array

        return grid

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

    def mark_obstacle_cell(self, x, y):
        self.grid_array[x, y] = 1

    def mark_free_cell(self, x, y):
        self.grid_array[x, y] = 0
        
    def mark_start_cell(self, x, y):
        self.grid_array[x, y] = 7
        
    def mark_goal_cell(self, x, y):
        self.grid_array[x, y] = 9
        
    def mark_robot_cell(self, x, y):
        self.grid_array[x, y] = 8

    def clear(self):
        self.grid_array = np.zeros([self.num_cols, self.num_rows])

    def get_obstacles(self):
        return zip(*np.where(self.grid_array == 1))
    
    def get_start(self):
        return zip(*np.where(self.grid_array == 7))
    
    def get_goal(self):
        return zip(*np.where(self.grid_array == 9))
    
    def get_robot(self):
        return zip(*np.where(self.grid_array == 8))

    def cell_xy(self, ix, iy):
        """Returns the center xy point of the cell."""
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits
        width, height = self.cell_dimensions
        return minx + (ix+0.5) * width, miny + (iy+0.5) * height

    def cell_verts(self, ix, iy):
        width, height = self.cell_dimensions
        x, y = self.cell_xy(ix, iy)
        verts = [(x + ofx*0.5*width, y + ofy*0.5*height) for ofx, ofy in [(-1,-1),(-1,1),(1,1),(1,-1)]]
        return verts

    def export_to_dict(self):
        export_dict = {}
        export_dict['grid'] = self.grid_array.tolist()
        return export_dict

    def load_from_dict(self, grid_dict):
        self.grid_array = np.array(grid_dict['grid'])

    def draw(self):
        cols, rows = self.size
        minx, maxx = self.xlimits
        miny, maxy = self.ylimits

        width, height = self.cell_dimensions

        x = map(lambda i: minx + width*i, range(cols+1))
        y = map(lambda i: miny + height*i, range(rows+1))

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
        self.draw_obstacles(plt.gca())
        self.draw_start_goal(plt.gca())
        self.draw_robot(plt.gca())

        return plt.gca()

    def draw_obstacles(self, axes):
        verts = [self.cell_verts(ix, iy) for ix,iy in self.get_obstacles()]
        collection_recs = PolyCollection(verts, facecolors='r')
        axes.add_collection(collection_recs)
        
    def draw_start_goal(self, axes):
        start_verts = [self.cell_verts(ix, iy) for ix,iy in self.get_start()]
        goal_verts = [self.cell_verts(ix, iy) for ix,iy in self.get_goal()]
        collection_recs = PolyCollection(start_verts, facecolors='b')
        axes.add_collection(collection_recs)
        collection_recs = PolyCollection(goal_verts, facecolors='g')
        axes.add_collection(collection_recs)
        
    def draw_robot(self, axes):
        verts = [self.cell_verts(ix, iy) for ix,iy in self.get_robot()]
        collection_recs = PolyCollection(verts, facecolors='pink')
        axes.add_collection(collection_recs)
        
    def draw_cell_circle(self, axes, xy, size=0.5, **kwargs):
        ix, iy = xy
        x, y = self.cell_xy(ix, iy)
        xr, yr = 0.5 * self.cell_dimensions[0], 0.5 * self.cell_dimensions[1]
        axes.add_patch(Ellipse((x,y), xr, yr, **kwargs))

    def draw_path(self, axes, path, *args, **kwargs):
        xy_coords = map(lambda idx: self.cell_xy(*idx), path)
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
        obstacles = self.get_obstacles()
        for col in range(xmin, xmax):
            for row in range(ymin, ymax):
                # For obstacle blocks, don't add any edges at all (I was considering
                # adding infinity-weight edges, but it just adds more work for the path
                # solver)
                if (col, row) not in obstacles:
                    for neighbor in filter(lambda n: n not in obstacles, neighbors((col,row))):
                        graph.add_edge((col,row), neighbor, bidirectional=False)
        
        return graph
                
                
                
                
                
                
                
                
                
                
                