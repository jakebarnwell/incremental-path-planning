from utils import PriorityQueue

INF = float("inf")

#todo this currently only supports undirected graphs

def dstar_lite(problem):

    #todo please stop doing gross things with global-ish variables
    start, goal, graph, queue, key_modifier, heuristic, g, rhs = [None]*8

    def calc_key(node):
        "Returns key as a tuple of two ints"
        min_g_rhs = min([g[node], rhs[node]])
        return (min_g_rhs + heuristic((start, node)) + key_modifier, min_g_rhs)

    def update_vertex(node):
        if node != goal:
            rhs[node] = min([graph.edge_len(node, neighbor) + g[neighbor] #todo Graph.edge_len
                             for neighbor in graph.get_neighbors(node)]) #todo Graph.get_neighbors
        if node in queue:
            queue.remove(node) #todo Queue.remove
        if g[node] != rhs[node]:
            queue.insert((node, calc_key(node)))

    def compute_shortest_path():
#        while True:
        for x in range(10): #todo change to while True (this is temporary to prevent infinite looping)
            smallest_key = queue.preview() #todo Queue.preview
            if smallest_key >= calc_key(start) and rhs[start] == g[start]:
                return
            node = queue.pop()[0] #todo store nodes in queue, not (node, key) pairs
            if smallest_key < calc_key(node):
                queue.insert((node, calc_key(node)))
            elif g[node] > rhs[node]:
                g[node] = rhs[node]
                for next_node in graph.get_neighbors(node):
                    update_vertex(next_node)
            else:
                g[node] = INF
                for next_node in graph.get_neighbors(node) + [node]:
                    update_vertex(next_node)

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

    queue = PriorityQueue(f=lambda tup: tup[1])
    queue.insert((goal, calc_key(goal)))

    last_start = start

    # Begin algorithm
    compute_shortest_path()

    while start != goal:
        if g[start] == INF:
            return "no path found" #todo return something useful
        start = min(graph.get_neighbors(start),
                    key = lambda neighbor: (graph.edge_len(start, neighbor)
                                            + g[neighbor]))
        problem.update_robot_position(start)
        if problem.changed_edges:
            key_modifier = key_modifier + heuristic(last_start, start)
            last_start = start
            for edge in problem.changed_edges:
                update_vertex(edge.source)
                update_vertex(edge.target)
            compute_shortest_path()
    return problem #contains path traversed and other info





