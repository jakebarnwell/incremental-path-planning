from grid import Grid
from search_problem import IncrementalSearchProblem
from world import World
from dstar import dstar_lite

grid_str_simple = """0 0 0
                     S 1 0
                     0 1 0
                     0 1 0
                     0 0 G"""
grid_simple = Grid.create_from_str(grid_str_simple)
robot_start_simple = (0,3)
world_simple = World(grid_simple, robot_start_simple, vision_radius=1.5)
goal_simple = (2,0)

problem_simple = IncrementalSearchProblem(world_simple, robot_start_simple,
                                          goal_simple)
result_simple = dstar_lite(problem_simple)
print result_simple