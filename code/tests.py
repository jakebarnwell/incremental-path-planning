from grid import Grid
from search_problem import IncrementalSearchProblem, World
from dstar import dstar_lite

grid = Grid(3,5)
robot_start = (0,3)
world = World(grid, robot_start, vision_radius=1)
goal = (2,0)

problem = IncrementalSearchProblem(world, robot_start, goal)
result = dstar_lite(problem)
print result