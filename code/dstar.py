from utils import PriorityQueue

INF = float("inf")

#todo this currently only supports undirected graphs

def dstar_lite(problem):

    #todo please stop doing gross things with global-ish variables
    start, goal, graph, queue, key_modifier, heuristic, g, rhs = [None]*8

    def calc_key(node):
#        print 'CALC KEY',
        "Returns key as a tuple of two ints"
        min_g_rhs = min([g[node], rhs[node]])
#        print (min_g_rhs + heuristic((start, node)) + key_modifier, min_g_rhs)
        return (min_g_rhs + heuristic((start, node)) + key_modifier, min_g_rhs)

    def update_vertex(node):
#        print 'UPDATE VERTEX', node
        if node != goal:
            rhs[node] = min([graph.get_edge_weight(node, neighbor) + g[neighbor]
                             for neighbor in graph.get_neighbors(node)])
        if node in queue:
            queue.remove(node)
        if g[node] != rhs[node]:
            queue.insert(node)

    def compute_shortest_path():
        print '> COMPUTE SHORTEST PATH'
#        while True:
        for x in range(16): #todo change to while True (this is temporary to prevent infinite looping)
            smallest_key = queue.top_key()
            if smallest_key >= calc_key(start) and rhs[start] == g[start]:
                print '> exiting loop as intended'
                return
            node = queue.pop()
            print '> node:', node
            if smallest_key < calc_key(node):
                queue.insert(node)
            elif g[node] > rhs[node]:
                g[node] = rhs[node]
                for next_node in graph.get_neighbors(node):
                    update_vertex(next_node)
            else:
                g[node] = INF
                for next_node in graph.get_neighbors(node) + [node]:
                    update_vertex(next_node)
        print '> time out'

    # Initialize
    start = problem.start_node
    goal = problem.goal_node
    graph = problem.get_graph()

    g = {node:INF for node in graph.get_all_nodes()}
    rhs = {node:INF for node in graph.get_all_nodes()}
    rhs[goal] = 0
    key_modifier = 0
#    heuristic = {} #maps keys (node1, node2) to int values #todo
    heuristic = lambda node_pair: 0 #todo

    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.insert(goal)

    last_start = start

    # Begin algorithm
    compute_shortest_path()

    while start != goal:
        if g[start] == INF:
            return "no path found" #todo return something useful
        start = min(graph.get_neighbors(start),
                    key = lambda neighbor: (graph.get_edge_weight(start, neighbor)
                                            + g[neighbor]))
        old_graph = graph.copy()
        graph = problem.update_world(start)
        changed_edges = graph.get_changed_edges(old_graph)
        if changed_edges:
            key_modifier = key_modifier + heuristic(last_start, start)
            last_start = start
            for edge in changed_edges:
                update_vertex(edge.source)
                update_vertex(edge.target)
            compute_shortest_path()
    return problem #contains path traversed and other info

