from grid import Grid
from graph import Graph
from search_problem import IncrementalSearchProblem
from world import World
from queue import PriorityQueue
import time

inf = float("inf")

def test_ok():
    """ If execution gets to this point, print out a happy message """
    try:
        from IPython.display import display_html
        display_html("""<div class="alert alert-success">
        <strong>Tests passed!!</strong>
        </div>""", raw=True)
    except:
        print "Tests passed!!"

#### PROBLEMS AND GRAPHS FOR TESTING ###########################################

# simple grid problem (from 10-page D* Lite paper, IEEE 2002, page 6)
grid_str_simple = """
0 0 0
S 1 0
0 1 0
0 1 0
0 0 G
"""

grid_simple = Grid.create_from_str(grid_str_simple)
robot_start_simple = (0,3)
world_simple = World(grid_simple, robot_start_simple, vision_radius=1.5)
goal_simple = (2,0)
problem_simple = IncrementalSearchProblem(world_simple, robot_start_simple,
                                          goal_simple)
graph_simple = problem_simple.get_graph()

problem_simple_step2 = problem_simple.copy()
graph_simple_step2 = problem_simple_step2.update_world([(0,2), (1,1), (2,0)])

problem_simple_complete = problem_simple_step2.copy()
problem_simple_complete.update_world([(0,1), (1,0), (2,0)])
problem_simple_complete.update_world([(1,0), (2,0)])
problem_simple_complete.update_world([(2,0)])

# triangle problem with directed graph
graph_triangle = Graph()
graph_triangle._nodes = set("ABC")
graph_triangle.add_edge("A", "B", 2, bidirectional=False)
graph_triangle.add_edge("B", "C", 3, bidirectional=False)
graph_triangle.add_edge("A", "C", 7, bidirectional=False)

# hard grid problem (from 16.410 pset)
#with open("grids/hard.txt", "r") as f:
#    grid_str_hard = f.read()
grid_str_hard = """
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 S 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 1 1 0 0 0
0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 1 0 0 0 1 1 0 0 0
0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 1 1 1 0 1 1
0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 1 1 1 1 0 0 1 0 0 0 0 0 0 0 1
0 1 1 1 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 1 1 1 1
0 1 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 1 1 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0
0 0 1 1 1 1 0 0 0 0 0 0 0 1 1 1 0 0 0 1 1 1 1 0 0 0 0 1 1 0 0 1 1 1 0 0 1 1 1 0
0 0 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 1 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0 0 0 1 0
0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 1 0 0 0 0 0 0 1 0
0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 1 1 0 0 1 0 0 0 0 0 0 1 0
0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0
0 0 0 0 1 1 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0 0 1 0
0 0 0 1 1 1 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 0
0 0 0 0 0 0 0 0 0 1 0 1 1 1 0 0 0 1 1 0 0 0 0 0 0 1 1 1 1 1 0 1 0 0 0 0 0 0 1 0
0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1 0 0 0 0 1 0 0 0 0 0 1 1 1 0 0 0 0 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 1 1 1 1 1 0 1 1 1 1 1 1 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0
0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 0 0 0 0 1 0 0
0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 1 1 1 0 0
0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 G 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 1 1 1 1 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
"""

#print grid_str_hard
grid_hard = Grid.create_from_str(grid_str_hard)
robot_start_hard = (20,29) #grid is 40 columns by 30 rows
world_hard = World(grid_hard, robot_start_hard, vision_radius=3)
goal_hard = (34,4)
problem_hard = IncrementalSearchProblem(world_hard, robot_start_hard, goal_hard)

#### TESTS #####################################################################

def test_grid_heuristic(grid_heuristic):
    print "Testing grid_heuristic..."
    assert grid_heuristic((1,6), (3,6)) == 2
    assert grid_heuristic((3,6), (1,6)) == 2
    assert grid_heuristic((8,4), (8,4)) == 0
    assert grid_heuristic((2,5), (5,2)) == 3
    assert grid_heuristic((70,30), (80,60)) == 30
    assert grid_heuristic((-5,3.2), (2,4.9)) == 7
#    print "grid_heuristic tests passed!\n"

def test_calc_key_helper(calc_key_helper):
    print "Testing calc_key_helper..."
    assert calc_key_helper((1,1), {(1,1):20}, {(1,1):30}, (0,3), 100) == (122,20)
    assert calc_key_helper((1,1), {(1,1):30}, {(1,1):20}, (0,3), 100) == (122,20)
    assert calc_key_helper((0,3), {(0,3):2, (1,0):200, (0,1):1, "garbage":"nonsense"}, {(0,3):99, (600,600):7}, (600,600), 40) == (642,2)
    zero_heuristic = lambda x,y: 0
    sum_heuristic = lambda x,y: x+y
    assert calc_key_helper("nodeA", {"nodeA":21}, {"nodeA":33}, "S", 500, zero_heuristic) == (521,21)
    assert calc_key_helper(30, {30:900}, {30:800}, 4, 6000, sum_heuristic) == (6834,800)
#    print "calc_key_helper tests passed!\n"


def test_update_vertex_helper(update_vertex_helper):
    print "Testing update_vertex_helper..."

    # Test 1: triangle problem; A not in queue, don't insert
    g = dict(A=9, B=10, C=2)
    g_original = g.copy()
    rhs = dict(A="value_to_be_overwritten")
    graph = graph_triangle.copy()
    queue = PriorityQueue() #empty
    update_vertex_helper("A", g, rhs, "non-existent_goal_node", graph, queue)
    assert g == g_original #not modified
    assert rhs == dict(A=9)
    assert graph == graph_triangle #not modified
    assert queue == PriorityQueue()

    # Test 2: triangle problem; remove A from queue and don't re-insert
    g = dict(A=9, B=10, C=2)
    g_original = g.copy()
    rhs = dict(A="value_to_be_overwritten")
    graph = graph_triangle.copy()
    queue = PriorityQueue() #empty
    queue.extend(list("XYAZ"))
    expected_queue = queue.copy()
    expected_queue.remove("A")
    update_vertex_helper("A", g, rhs, "non-existent_goal_node", graph, queue)
    assert g == g_original #not modified
    assert rhs == dict(A=9)
    assert graph == graph_triangle #not modified
    assert queue == expected_queue

    # Test 3: triangle problem; A not in queue, do insert
    g = dict(A=8, B=10, C=2)
    g_original = g.copy()
    rhs = dict(A=0)
    graph = graph_triangle.copy()
    queue = PriorityQueue() #empty
    queue.extend(list("XYZ"))
    expected_queue = queue.copy()
    expected_queue.insert("A")
    update_vertex_helper("A", g, rhs, "non-existent_goal_node", graph, queue)
    assert g == g_original #not modified
    assert rhs == dict(A=9)
    assert graph == graph_triangle #not modified
    assert queue == expected_queue

    # Test 4: triangle problem; A in queue, remove and re-insert
    g = dict(A=20, B=10, C=2)
    g_original = g.copy()
    rhs = dict(B="other_stuff")
    graph = graph_triangle.copy()
    queue = PriorityQueue() #empty
    queue.extend(list("XYAZ"))
    expected_queue = queue.copy()
    update_vertex_helper("A", g, rhs, "non-existent_goal_node", graph, queue)
    assert g == g_original #not modified
    assert rhs == dict(A=9, B="other_stuff")
    assert graph == graph_triangle #not modified
    assert queue == expected_queue

    # Test 5: triangle problem; B has only one successor
    g = dict(A=1, B=100, C=10)
    g_original = g.copy()
    rhs = dict()
    queue = PriorityQueue() #empty
    expected_queue = queue.copy()
    expected_queue.insert("B")
    update_vertex_helper("B", g, rhs, "non-existent_goal_node",
                         graph_triangle.copy(), queue)
    assert rhs == dict(B=13)
    assert queue == expected_queue

    # Test 6: triangle problem; B is goal
    g = dict(A=1, B=0, C=10)
    g_original = g.copy()
    rhs = dict(B=0)
    queue = PriorityQueue() #empty
    update_vertex_helper("B", g, rhs, "B", graph_triangle.copy(), queue)
    assert rhs == dict(B=0)
    assert queue == PriorityQueue()

    # Test 7: bidirectional graph from problem_simple
    g = {(1, 2): inf, (0, 1): inf, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 2.0, (2, 0): 0, (1, 3): 5, (2, 3): inf, (2, 1): 1.0, (2, 2): inf, (1, 0): inf, (4, 2): inf, (0, 3): inf, (0, 4): inf, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    rhs = {(1, 2): inf, (0, 1): 2.0, (3, 2): inf, (0, 0): 2.0, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 2.0, (2, 0): 0, (1, 3): 9, (2, 3): inf, (2, 1): 1.0, (2, 2): 2.0, (1, 0): 1.0, (4, 2): inf, (0, 3): inf, (0, 4): inf, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    queue = PriorityQueue()
    queue.extend([(1, 0), (0, 1), (2, 2), (0, 0)])
    expected_rhs = {(1, 2): inf, (0, 1): 2.0, (3, 2): inf, (0, 0): 2.0, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 2.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): 2.0, (1, 0): 1.0, (4, 2): inf, (0, 3): inf, (0, 4): inf, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    expected_queue = queue.copy()
    expected_queue.insert((1,3))
    update_vertex_helper((1,3), g, rhs, goal_simple, graph_simple, queue)
    assert rhs == expected_rhs
    assert queue == expected_queue

#    print "update_vertex_helper tests passed!\n"


def test_compute_shortest_path_helper(compute_shortest_path_helper,calc_key_helper):
    print "Testing compute_shortest_path_helper..."

    # problem_simple, first step
    g = {(1, 2): inf, (0, 1): inf, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): inf, (2, 0): inf, (1, 3): inf, (2, 3): inf, (2, 1): inf, (2, 2): inf, (1, 0): inf, (4, 2): inf, (0, 3): inf, (0, 4): inf, (4, 1): inf, (1, 1): inf, (4, 0): inf}
    rhs = {(1, 2): inf, (0, 1): inf, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): inf, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): inf, (2, 2): inf, (1, 0): inf, (4, 2): inf, (0, 3): inf, (0, 4): inf, (4, 1): inf, (1, 1): inf, (4, 0): inf}
    graph = graph_simple.copy()
    calc_key = lambda node: calc_key_helper(node, g, rhs, (0, 3), 0)
    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.insert(goal_simple)
    g_expected = {(1, 2): inf, (0, 1): inf, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 2.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): inf, (1, 0): inf, (4, 2): inf, (0, 3): 3.0, (0, 4): inf, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    rhs_expected = {(1, 2): inf, (0, 1): 2.0, (3, 2): inf, (0, 0): 2.0, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): 4.0, (0, 2): 2.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): 2.0, (1, 0): 1.0, (4, 2): inf, (0, 3): 3.0, (0, 4): 4.0, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    compute_shortest_path_helper(g, rhs, (0, 3), goal_simple, 0, graph, queue)
    assert graph == graph_simple #not modified
    assert g == g_expected
    assert rhs == rhs_expected
    queue_expected = PriorityQueue(f=lambda node: calc_key(node))
    queue_expected.extend([(1, 0), (0, 1), (2, 2), (0, 0), (0, 4), (1, 4)])
    assert queue == queue_expected

    # problem_simple, second step
    g = {(1, 2): inf, (0, 1): inf, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 2.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): inf, (1, 0): inf, (4, 2): inf, (0, 3): 3.0, (0, 4): inf, (4, 1): inf, (1, 1): 1.0, (4, 0): inf}
    rhs = {(1, 2): inf, (0, 1): 3.0, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): 4.0, (0, 2): 4.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): 2.0, (1, 0): 1.0, (4, 2): inf, (0, 3): 3.0, (0, 4): 4.0, (4, 1): inf, (1, 1): inf, (4, 0): inf}
    graph = graph_simple_step2.copy()
    calc_key = lambda node: calc_key_helper(node, g, rhs, (0, 2), 1)
    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.extend([(1, 1), (0, 2), (1, 0), (2, 2), (0, 1), (0, 4), (1, 4)])
    g_expected = {(1, 2): inf, (0, 1): 2.0, (3, 2): inf, (0, 0): inf, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): inf, (0, 2): 3.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): inf, (1, 0): 1.0, (4, 2): inf, (0, 3): 3.0, (0, 4): inf, (4, 1): inf, (1, 1): inf, (4, 0): inf}
    rhs_expected = {(1, 2): inf, (0, 1): 2.0, (3, 2): inf, (0, 0): 2.0, (3, 0): inf, (3, 1): inf, (2, 4): inf, (1, 4): 4.0, (0, 2): 3.0, (2, 0): 0, (1, 3): inf, (2, 3): inf, (2, 1): 1.0, (2, 2): 2.0, (1, 0): 1.0, (4, 2): inf, (0, 3): 4.0, (0, 4): 4.0, (4, 1): inf, (1, 1): inf, (4, 0): inf}
    compute_shortest_path_helper(g, rhs, (0, 2), goal_simple, 1, graph, queue)
    assert graph == graph_simple_step2 #not modified
    assert g == g_expected
    assert rhs == rhs_expected
    queue_expected = PriorityQueue(f=lambda node: calc_key(node))
    queue_expected.extend([(0, 0), (2, 2), (0, 3), (0, 4), (1, 4)])
    assert queue == queue_expected

#    print "compute_shortest_path_helper tests passed!\n"


def test_dstar_lite(dstar_lite):
    print "Testing dstar_lite on a 3x5 grid..."
#    print grid_str_simple
    problem = problem_simple.copy()
    result = dstar_lite(problem)
    problem._world.draw_all_path()
    assert result == problem_simple_complete
    assert problem == problem_simple_complete #check that problem was modified
#    print "dstar_lite test passed!\n"

def run_dstar_lite_hard_grid(dstar_lite):
    print "Running dstar_lite on a 40x30 grid..."
#    print grid_str_hard
    start_time = time.time()
    problem = problem_hard.copy()
    dstar_lite(problem)
    end_time = time.time()
    print "Robot reached goal in %s seconds\n" % str(end_time - start_time)
    problem._world.draw_all_path()

