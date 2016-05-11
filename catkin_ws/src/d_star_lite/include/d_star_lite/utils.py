from __future__ import division

import math
import numpy as np
from d_star_lite.world import World
from d_star_lite.grid import *
from d_star_lite.queue import PriorityQueue
from d_star_lite.graph import get_intended_path

inf = float("inf")

def grid_heuristic(node1, node2):
    """Given two nodes as (x,y) grid-coordinate tuples (e.g. (2,3)), computes the
    heuristic grid-based heuristic value between the nodes.
    (Hint: The heuristic value is just the maximum of the difference in x or y.)"""
    return max(map(abs, [node1[i]-node2[i] for i in (0,1)]))

def euclidean_heuristic(node1, node2):
    return np.linalg.norm(np.array(node2) - np.array(node1))

def square_euclidean_heuristic(node1, node2):
    s = min(map(abs, [node1[i]-node2[i] for i in (0,1)]))
    l = max(map(abs, [node1[i]-node2[i] for i in (0,1)])) - s
    return s*math.sqrt(2) + l

def calc_key_helper(node, g, rhs, start, key_modifier, heuristic=grid_heuristic):
    "Computes the node's current key and returns it as a tuple of two numbers."
    min_g_rhs = min([g[node], rhs[node]])
    return (min_g_rhs + heuristic(start, node) + key_modifier, min_g_rhs)

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

def compute_shortest_path_helper(g, rhs, start, goal, key_modifier, graph, queue, _heuristic):
    """As in the D* Lite pseudocode, this method computes (or recomputes) the
    shortest path by popping nodes off the queue, updating their g and rhs
    values, and calling update_vertex on their neighbors.  Returns nothing."""
    # Helper functions that take in only one argument, node:
    def calc_key(node):
        return calc_key_helper(node, g, rhs, start, key_modifier, heuristic=_heuristic)
    def update_vertex(node):
        return update_vertex_helper(node, g, rhs, goal, graph, queue)

    while True:
        smallest_key = queue.top_key()
        if smallest_key >= calc_key(start) and rhs[start] == g[start]:
            break
        node = queue.pop()
        if smallest_key < calc_key(node):
            print smallest_key, calc_key(node), node
            queue.insert(node)
        elif g[node] > rhs[node]:
            g[node] = rhs[node]
            for next_node in graph.get_predecessors(node):
                update_vertex(next_node)
        else:
            g[node] = inf
            for next_node in graph.get_predecessors(node) + [node]:
                update_vertex(next_node)
        if queue.is_empty():
            break

def resolve_point_to_node_helper(point, graph, grid_res,map_displacement):
    #return min(graph.get_all_nodes(), key = lambda node: math.sqrt((node[0]-point.x)**2 + (node[1]-point.y)**2))
    node_x = int(round((point.x-map_displacement.x)/grid_res))
    node_y = int(round((point.y-map_displacement.y)/grid_res))
    return (node_x,node_y)

def convert_node_to_point_helper(node, grid_res,map_displacement):
    x = node[0]*grid_res+map_displacement.x
    y = node[1]*grid_res+map_displacement.y
    return (x,y)
