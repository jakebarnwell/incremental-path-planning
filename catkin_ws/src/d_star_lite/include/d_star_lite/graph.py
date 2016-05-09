import networkx as nx
import matplotlib.pyplot as plt
from copy import deepcopy
import numpy as np

def get_intended_path(next_step, goal, graph, g): #for D* Lite
    """Uses g-values to reconstruct future planned path given intended next
    step.  Returns a path as a list [next_step, ... , goal]"""
    path = [next_step]
    while path[-1] != goal:
        path.append(min(graph.get_successors(path[-1]),
                        key=lambda node: g[node]))
    return path

class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % self.node


class Edge(object):
    def __init__(self, source, target, weight=1.0):
        self.source = source
        self.target = target
        self.weight = weight

    def __hash__(self):
        return hash("%s_%s_%f" % (self.source, self.target, self.weight))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target \
            and self.weight == other.weight
    def __repr__(self):
        return "Edge(%r,%r,%r)" % (self.source, self.target, self.weight)
    __str__ = __repr__


class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = set()
        self._edges = dict() #maps each source node to a set containing its outgoing edges
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __eq__(self, other):
        return self._nodes == other._nodes and self._edges == other._edges

    def __repr__(self):
        return "Graph<nodes: %s, edges: %s>" % (str(self._nodes), str(self._edges))
    __str__ = __repr__

    def __contains__(self, node):
        return node in self._nodes

    @classmethod
    def fromArray(grid_class, numpy_2d_array, set_viz_data=True):
        """
        Creates a Graph object from a 2D numpy array. The 2d numpy
        array must be at least 1x1. The entries of the array must
        be either 0 or 1.
        Returns the Graph object created.
        Enable the set_viz_data flag if you want to allow for the
        visualization/drawing of this graph. True is the default. However,
        in production code, you may want to set this flag to False to
        speed up the creation of this Graph.
        This method should be reasonably efficient.
        """
        # A lot of this is copied from the grid.to_graph method

        # Some easy arrays to test this function on:
        #   np.array([[0,1,1,0],[1,0,1,1],[1,1,0,0],[0,1,0,1],[0,0,1,1]])
        #   np.array([[1,0,0,0],[1,0,0,0],[0,0,0,1],[0,0,0,1],[0,0,0,1]])
        # The visualization code should show these as written, i.e.
        #  5x4 matrices (5 rows, 4 columns), with the origin being top
        #  left.

        # Make sure this is a 2D array
        assert len(numpy_2d_array.shape) == 2
        height, width = numpy_2d_array.shape

        _DELTA = [(1,1),(0,1),(-1,1),(-1,0)]
        def efficient_neighbors(row, col):
            """
            Returns the *valid* north-east, east, south-east, and south neighbors
            of this cell as a list of tuples. If any such neighbors don't
            exist (e.g. because the current node is on the right edge) the
            given neighbor is not included.
            """
            n = [(row + d[0], col + d[1]) for d in _DELTA]
            n = filter(lambda nn: 0<=nn[0]<height and 0<=nn[1]<width, n)
            return n

        graph = grid_class()

        # Add all numpy array indices as nodes. I hate repeating code but
        #  I separate the two cases, since I prefer not to put if-statements
        #  inside the loop; maybe I'm being dumb about this.
        if set_viz_data:
            node_positions = {}
            for col in range(width):
                for row in range(height):
                    graph.add_node((col,row))
                    # To add the physical drawing positions, we
                    #  must negate the row, since the drawing
                    #  code considers nodes as
                    #      (x,y), origin at bottom left
                    # instead of
                    #      (x,y), origin at top left
                    node_positions[(col,row)] = (col,-row)
            graph.set_node_positions(node_positions)
        else:
            for col in range(width):
                for row in range(height):
                    graph.add_node((col,row))

        # Add bi-directional edges between the 8 cardinal+ordinal neighbors.
        #  We need to do this in an efficient way, so let's go top left to
        #  bottom right, adding bi-directional edges to each of the NE, E, SE,
        #  and S neighbors.23
        obstacles = zip(*np.where(numpy_2d_array == 1))
        _WEIGHTS = [1, np.inf, np.inf] # Cardinal weights
        _WEIGHTS_d = [np.sqrt(2), np.inf, np.inf] # Oriental weights
        for c in range(width):
            for r in range(height):
                # From this node (c,r), add up to 4 bidirectional edges
                #  to the efficient neighbors; the edges have weight 1, sqrt(2),
                #  or infty depending on if this node or the neighbor node (or
                #  both, or neither) is an obstacle.
                for n in efficient_neighbors(r, c):
                    if r == n[0] or c == n[1]:
                        graph.add_edge((c,r), (n[1],n[0]), weight=_WEIGHTS[numpy_2d_array[r,c]+numpy_2d_array[n[0],n[1]]], bidirectional=True)
                    else:
                        graph.add_edge((c,r), (n[1],n[0]), weight=_WEIGHTS_d[numpy_2d_array[r,c]+numpy_2d_array[n[0],n[1]]], bidirectional=True)
                    # Note the use of addition of the two numpy entry values
                    #  instead of multlipication: Possible values using addition
                    #  are {0,1,2} while possible values doing multiplication are
                    #  {0,1}. However, addition is about 10% faster...

        return graph

    def get_all_nodes(self):
        return self._nodes.copy()

    def add_node(self, node):
        """Adds a node to the graph."""
        self._nodes.add(node)

    def add_edge(self, node1, node2, weight=1.0, bidirectional=True):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2, weight))
        self._edges[node1] = node1_edges
        if bidirectional:
                node2_edges = self._edges.get(node2, set())
                node2_edges.add(Edge(node2, node1, weight))
                self._edges[node2] = node2_edges

    def set_node_positions(self, positions):
        self.node_positions = positions

    def set_node_pos(self, node, pos):
        """Sets the (x,y) pos of the node, if it exists in the graph."""
        if not node in self:
            raise NodeNotInGraph(node)
        self.node_positions[node] = pos

    def get_node_pos(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_positions[node]

    def get_outgoing_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())

    def get_successors(self, node):
        return map(lambda edge: edge.target, self.get_outgoing_edges(node))

    def get_predecessors(self, node):
        predecessors = []
        for source in self._edges:
            predecessors.extend(map(lambda edge: edge.source,
                                    filter(lambda edge: edge.target==node,
                                           self.get_outgoing_edges(source))))
        return predecessors

    def get_edge(self, source, target):
        """Returns the Edge connecting source to target, or None if no such Edge
        exists.  Assumes that at most one such edge exists."""
        matching_edges = filter(lambda edge: edge.target == target,
                                self.get_outgoing_edges(source))
        return matching_edges[0] if matching_edges else None

    def get_edge_weight(self, source, target):
        edge = self.get_edge(source, target)
        if edge is None:
            raise ValueError("There is no edge from %s to %s"
                             % map(str, (source, target)))
        return edge.weight

    def get_changed_edges(self, other_graph):
        """returns a set of tuples (my_edge, their_edge) containing
        corresponding pairs of edges whose weights differ between the two graphs.
        If an edge exists in only one graph, one value of the tuple will be None."""
        my_edges = deepcopy(self._edges)
        their_edges = deepcopy(other_graph._edges)
        changed_edges = set()
        for source in my_edges:
            if source not in their_edges:
                for edge in my_edges[source]:
                    changed_edges.add((edge, None))

            for edge in my_edges[source]:
                if edge in their_edges[source]:
                    their_edges[source].remove(edge)
                else:
                    their_corresponding_edge = other_graph.get_edge(edge.source, edge.target)
                    if their_corresponding_edge is not None:
                        changed_edges.add((edge, their_corresponding_edge))
                        their_edges[source].remove(their_corresponding_edge)
                    else:
                        changed_edges.add((edge, None))
        for source in their_edges:
            for remaining_edge in their_edges[source]:
                changed_edges.add((None, remaining_edge))
        return changed_edges

    def copy(self):
        return deepcopy(self)

    # DRAWING METHODS
    #def draw(self, highlight_edges=None):
    #    nxg = nx.DiGraph()
    #    edges = [(e.source, e.target, {'weight':e.weight, 'inv_weight':1.0/e.weight}) for node_set in self._edges.values() for e in node_set]
    #    nxg.add_edges_from(edges)
    #    if len(self.node_positions) < len(self._nodes):
    #        # Calculate positions for nodes whose pos is not specified.
    #        pos = nx.spring_layout(nxg, weight='inv_weight', pos=self.node_positions, fixed=self.node_positions.keys() if self.node_positions else None)
    #    else:
    #        pos = self.node_positions

    #    f = plt.figure(figsize=(12,12))
    #    plt.gca().set_aspect('equal', adjustable='box')
    #    nx.draw_networkx_nodes(nxg, pos, node_color='w')
    #    nx.draw_networkx_edges(nxg, pos, edges)
    #    nx.draw_networkx_labels(nxg, pos)
    #    edge_labels=dict([((u,v,),"%s" % d['weight'])
    #             for u,v,d in nxg.edges(data=True)])
    #    nx.draw_networkx_edge_labels(nxg, pos, edge_labels=edge_labels)


    #    if highlight_edges:
    #        nx.draw_networkx_edges(nxg, pos, highlight_edges, edge_color='r')

    #    plt.axis('off')
    #    plt.show()

    #def draw_edges(self, edges):
    #    nx.draw_networkx_edges(nxg, pos, edges, edge_color='r')
    #    reduced_labels = {(u,v): edge_labels[(u,v)] for u,v,_ in edges}
    #    nx.draw_networkx_edge_labels(nxg, pos, edge_labels=reduced_labels, font_color='r')

    #    reduced_nodes = set([u for u,_,_ in edges])
    #    reduced_nodes.update([v for _,v,_ in edges])
    #    # nx.draw_networkx_nodes(nxg, pos, nodelist=reduced_nodes,  node_color='r')
    #    red_labels = {n:n for n in reduced_nodes}
    #    print red_labels
    #    nx.draw_networkx_labels(nxg, pos, labels=red_labels, font_color='r')

    def highlight_edges(self, edges):
        nx.draw_networkx_edges(nxg, pos, edges, edge_color='r')
        reduced_labels = {(u,v): edge_labels[(u,v)] for u,v,_ in edges}
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=reduced_labels, font_color='r')

        reduced_nodes = set([u for u,_,_ in edges])
        reduced_nodes.update([v for _,v,_ in edges])
        # nx.draw_networkx_nodes(nxg, pos, nodelist=reduced_nodes,  node_color='r')
        red_labels = {n:n for n in reduced_nodes}
        print red_labels
        nx.draw_networkx_labels(nxg, pos, labels=red_labels, font_color='r')
