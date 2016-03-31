from grid import Grid
from search_problem import IncrementalSearchProblem
from world import World
from dstar import dstar_lite

grid_str = """0 0 0
              S 1 0
              0 1 0
              0 1 0
              0 0 G"""
grid = Grid.create_from_str(grid_str)
robot_start = (0,3)
world = World(grid, robot_start, vision_radius=1)
goal = (2,0)

problem = IncrementalSearchProblem(world, robot_start, goal)
result = dstar_lite(problem)
print result