from queue import PriorityQueue
from utils import get_intended_path

inf = float("inf")

# HELPER FUNCTIONS

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
    return max(map(abs, [node1[i]-node2[i] for i in (0,1)]))

def calc_key_helper(node, g, rhs, start, key_modifier, heuristic=grid_heuristic):
    "Computes the node's current key and returns it as a tuple of two numbers."
    min_g_rhs = min([g[node], rhs[node]])
    return (min_g_rhs + heuristic(start, node) + key_modifier, min_g_rhs)

#todo move to tests
def test_grid_heuristic():
    print "Testing grid_heuristic..."
    assert grid_heuristic((1,6), (3,6)) == 2
    assert grid_heuristic((3,6), (1,6)) == 2
    assert grid_heuristic((8,4), (8,4)) == 0
    assert grid_heuristic((2,5), (5,2)) == 3
    assert grid_heuristic((70,30), (80,60)) == 30
    assert grid_heuristic((-5,3.2), (2,4.9)) == 7
    print "grid_heuristic tests passed!"

def test_calc_key_helper():
    print "Testing calc_key_helper..."
    assert calc_key_helper((1,1), {(1,1):20}, {(1,1):30}, (0,3), 100) == (122,20)
    assert calc_key_helper((1,1), {(1,1):30}, {(1,1):20}, (0,3), 100) == (122,20)
    assert calc_key_helper((0,3), {(0,3):2, (1,0):200, (0,1):1, "garbage":"nonsense"}, {(0,3):99, (600,600):7}, (600,600), 40) == (642,2)
    zero_heuristic = lambda x,y: 0
    sum_heuristic = lambda x,y: x+y
    assert calc_key_helper("nodeA", {"nodeA":21}, {"nodeA":33}, "S", 500, zero_heuristic) == (521,21)
    assert calc_key_helper(30, {30:900}, {30:800}, 4, 6000, sum_heuristic) == (6834,800)
    print "calc_key_helper tests passed!"

# Test grid_heuristic and calc_key_helper:
test_grid_heuristic()
test_calc_key_helper()


"""Next, implement update_vertex_helper, following the pseudocode for D* Lite,
then run the tests below."""

def update_vertex_helper(node, g, rhs, goal, graph, queue):
    """As in the D* Lite pseudocode, this method updates node's rhs value and
    queue status. Returns nothing."""
    if node != goal:
        rhs[node] = min([graph.get_edge_weight(node, neighbor) + g[neighbor]
                         for neighbor in graph.get_successors(node)])
    if node in queue:
        queue.remove(node)
    if g[node] != rhs[node]:
        queue.insert(node)

# Test update_vertex_helper:
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
        update_vertex_helper(node, g, rhs, goal, graph, queue)

    # Your code here
    print '> COMPUTE SHORTEST PATH'
    while True:
        smallest_key = queue.top_key()
        if smallest_key >= calc_key(start) and rhs[start] == g[start]:
            return
        node = queue.pop()
        print '> dequeue node', node, 'with h =', grid_heuristic(node, start)
        if smallest_key < calc_key(node):
            queue.insert(node)
        elif g[node] > rhs[node]:
            g[node] = rhs[node]
            for next_node in graph.get_predecessors(node):
                update_vertex(next_node)
        else:
            g[node] = inf
            for next_node in graph.get_predecessors(node) + [node]:
                update_vertex(next_node)

# Test compute_shortest_path_helper:
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
        update_vertex_helper(node, g, rhs, goal, graph, queue)
    def compute_shortest_path():
        compute_shortest_path_helper(g, rhs, start, goal, key_modifier, graph, queue)

    # Initialize the queue using the priority function calc_key
    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.insert(goal)

    ############################################################################
    # YOUR CODE HERE

    # Begin algorithm
    last_start = start
    compute_shortest_path()
    print 'robot starting at:', start

    while start != goal:
        if g[start] == inf:
            print "no path found"
            return problem
        start = min(graph.get_successors(start),
                    key = lambda neighbor: (graph.get_edge_weight(start, neighbor)
                                            + g[neighbor]))
        old_graph = graph.copy()
        print 'robot moving to:', start
        intended_path = get_intended_path(start, goal, graph, g)
        print 'intended path:', intended_path
        graph = problem.update_world(intended_path)
        changed_edges = old_graph.get_changed_edges(graph)
        if changed_edges:
            key_modifier = key_modifier + grid_heuristic(last_start, start)
            last_start = start
            for (old_edge, new_edge) in changed_edges:
                if old_edge and new_edge: #edge simply changed weight
                    update_vertex(old_edge.source)
                elif not old_edge: #new edge was added
                    raise NotImplementedError("Edge addition not yet supported")
                else: #old edge was deleted
                    raise NotImplementedError("Edge deletion not yet supported")
            compute_shortest_path()
    print 'robot at:', start
    return problem #contains path traversed, intended future path, and other info

# Test dstar_lite
#test_dstar_lite()
