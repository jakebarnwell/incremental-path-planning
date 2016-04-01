from queue import PriorityQueue

inf = float("inf")

#todo this currently only supports undirected graphs

def dstar_lite(problem):

    #todo please stop doing gross things with global-ish variables
    start, goal, graph, queue, key_modifier, heuristic, g, rhs = [None]*8

    def calc_key(node):
        "Returns key as a tuple of two ints"
        min_g_rhs = min([g[node], rhs[node]])
        return (min_g_rhs + heuristic(start, node) + key_modifier, min_g_rhs)

    def update_vertex(node):
        if node != goal:
            rhs[node] = min([graph.get_edge_weight(node, neighbor) + g[neighbor]
                             for neighbor in graph.get_neighbors(node)])
        if node in queue:
            queue.remove(node)
        if g[node] != rhs[node]:
            queue.insert(node)

    def compute_shortest_path():
        print '> COMPUTE SHORTEST PATH'
        while True:
            smallest_key = queue.top_key()
            if smallest_key >= calc_key(start) and rhs[start] == g[start]:
                return
            node = queue.pop()
            print '> dequeue node', node, 'with h =', heuristic(node, start)
            if smallest_key < calc_key(node):
                queue.insert(node)
            elif g[node] > rhs[node]:
                g[node] = rhs[node]
                for next_node in graph.get_neighbors(node):
                    update_vertex(next_node)
            else:
                g[node] = inf
                for next_node in graph.get_neighbors(node) + [node]:
                    update_vertex(next_node)

    # Initialize
    start = problem.start_node
    goal = problem.goal_node
    graph = problem.get_graph()

    g = {node:inf for node in graph.get_all_nodes()}
    rhs = {node:inf for node in graph.get_all_nodes()}
    rhs[goal] = 0
    key_modifier = 0
    heuristic = lambda node1, node2: max((abs(node1[0]-node2[0]), abs(node1[1]-node2[1]))) #todo move to grid class?

    queue = PriorityQueue(f=lambda node: calc_key(node))
    queue.insert(goal)

    last_start = start

    # Begin algorithm
    compute_shortest_path()
    print 'robot starting at:', start

    while start != goal:
        if g[start] == inf:
            print "no path found"
            return problem
        start = min(graph.get_neighbors(start),
                    key = lambda neighbor: (graph.get_edge_weight(start, neighbor)
                                            + g[neighbor]))
        old_graph = graph.copy()
        print 'robot moving to:', start
        intended_path = build_intended_path(start, goal, graph, g)
        print 'intended path:', intended_path
        graph = problem.update_world(intended_path)
        changed_edges = old_graph.get_changed_edges(graph)
        if changed_edges:
            key_modifier = key_modifier + heuristic(last_start, start)
            last_start = start
            for (old_edge, new_edge) in changed_edges:
                if old_edge and new_edge: #edge simply changed weight
                    update_vertex(old_edge.source)
                    update_vertex(old_edge.target)
                elif not old_edge: #new edge was added
                    raise NotImplementedError("Edge addition not yet supported")
                else: #old edge was deleted
                    raise NotImplementedError("Edge deletion not yet supported")
            compute_shortest_path()
    print 'robot at:', start
    return problem #contains path traversed and other info

def build_intended_path(next_step, goal, graph, g_values):
    """Uses g-values to reconstruct future planned path given intended next
    step.  Returns a path as a list [next_step, ... , goal]"""
    path = [next_step]
    while path[-1] != goal:
        path.append(min(graph.get_neighbors(path[-1]),
                        key=lambda node: g_values[node]))
    return path
