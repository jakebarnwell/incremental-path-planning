from queue import PriorityQueue
from utils import get_intended_path

inf = float("inf")

"""
Our implementation of the algorithm uses the following variables:
g: a dictionary mapping nodes to their current g-values
rhs: a dictionary mapping nodes to their current rhs-values
key_modifier: a number representing the current key modifier k_m
queue: a PriorityQueue using calc_key as its priority function
graph: the Graph representing the robot's initial map of the world. You'll want
    to update this variable whenever the world changes.

First, you'll code some helper functions to be used in the D* Lite algorithm.

Implement grid_heuristic and calc_key_helper, then run the tests below.
"""

def grid_heuristic(node1, node2):
    """Given two nodes as (x,y) grid-coordinate tuples, computes the heuristic
    value between the nodes.  (Hint: The heuristic value is just the maximum of
    the difference in x or y.)"""
    raise NotImplementedError

def calc_key_helper(node, g, rhs, start, key_modifier, heuristic=grid_heuristic):
    "Computes the node's current key and returns it as a tuple of two numbers."
    raise NotImplementedError


"Test grid_heuristic and calc_key_helper:"
#test_grid_heuristic()
#test_calc_key_helper()


"""Next, implement update_vertex_helper, following the pseudocode for D* Lite,
then run the tests below."""

def update_vertex_helper(node, g, rhs, goal, graph, queue):
    """As in the D* Lite pseudocode, this method updates node's rhs value and
    queue status. Returns nothing."""
    raise NotImplementedError


"Test update_vertex_helper:"
#test_update_vertex_helper()


"""
Just one more helper function before we get to the main function!
Implement compute_shortest_path_helper, following the pseudocode for D* Lite,
then run the tests below.

Note that you can use the functions calc_key(node) and update_vertex(node) to
call your helper functions without needing to pass in all the arguments.
"""

def compute_shortest_path_helper(g, rhs, start, goal, key_modifier, graph, queue):
    """As in the D* Lite pseudocode, this method computes (or recomputes) the
    shortest path by popping nodes off the queue, updating their g and rhs
    values, and calling update_vertex on their neighbors.  Returns nothing."""
    # Helper functions that take in only one argument, node:
    def calc_key(node):
        return calc_key_helper(node, g, rhs, start, key_modifier)
    def update_vertex(node):
        return update_vertex_helper(node, g, rhs, goal, graph, queue)

    # YOUR CODE HERE
    raise NotImplementedError


"Test compute_shortest_path_helper:"
#test_compute_shortest_path_helper()


"""
Now it's time to implement D* Lite!

As above, you can use the functions calc_key(node) and update_vertex(node), as
well as compute_shortest_path(), to call your helper functions without needing
to pass in all the arguments.

As described in the API, the only way to update an IncrementalSearchProblem is
by calling problem.update_world(intended_path).  However, the standard D* Lite
algorithm only keeps track of the robot's next step, not its entire planned
path.  To get the planned path, you can use this helper function:

    get_intended_path(next_step, goal, graph, g)

It takes in a node next_step indicating where the robot will move to next (i.e.
since the last update), the goal node, the robot's current graph, and the dict
of g-values.  It uses g-values to reconstruct and return the robot's planned
path as a list of nodes [next_step, ... , goal], which is exactly the argument
required by problem.update_world.

In the method dstar_lite, below, we've implemented all the initialization for
you.  We recommend reading the comments in the INITIALIZE section to understand
what is going on.  Then, implement the rest of D* Lite, using the provided API
and following the D* Lite pseudocode.  When you're done, you can test your
implementation in the test block below.
"""

def dstar_lite(problem):
    """Performs D* Lite to find incrementally find a shortest path as the robot
    moves through the graph.  Updates the IncrementalSearchProblem, problem, by
    calling problem.update_world.  The search terminates when the robot either
    reaches the goal, or finds that there is no path to the goal.  Returns the
    modified problem.

    Note: The world is dynamic, so the true positions of obstacles may change as
    the robot moves through the world.  However, if the robot determines at any
    point that there is no finite path to the goal, it should stop searching and
    give up, rather than waiting and hoping that the world will improve.
    """

    ############################################################################
    # INITIALIZE

    # Get the start node, goal node, and graph from the IncrementalSearchProblem
    start = problem.start_node
    goal = problem.goal_node
    graph = problem.get_graph()

    # Set g=inf and rhs=inf for all nodes, except the goal node, which has rhs=0
    g = {node:inf for node in graph.get_all_nodes()}
    rhs = {node:inf for node in graph.get_all_nodes()}
    rhs[goal] = 0

    # Set the key modifier k_m to 0
    key_modifier = 0

    # Define shortened helper functions
    def calc_key(node):
        return calc_key_helper(node, g, rhs, start, key_modifier)
    queue = None # to be reinitialized later
    def update_vertex(node):
        return update_vertex_helper(node, g, rhs, goal, graph, queue)
    def compute_shortest_path():
        return compute_shortest_path_helper(g, rhs, start, goal, key_modifier, graph, queue)

    # Initialize the queue using the priority function calc_key
    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.insert(goal)

    ############################################################################
    # YOUR CODE HERE
    raise NotImplementedError

    return problem


"Test dstar_lite:"
#test_dstar_lite()
