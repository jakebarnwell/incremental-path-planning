#import pydot_ng as pydot
import networkx as nx
import matplotlib.pyplot as plt
from copy import deepcopy

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


class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = set()
        self._edges = dict()
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __contains__(self, node):
        return node in self._nodes

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

    def node_edges(self, node): #todo is this only outgoing edges?
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())

    def get_neighbors(self, node): #todo probably inefficient #todo this is undirected
        return map(lambda edge: edge.source if edge.source != node else edge.target,
                   self.node_edges(node))

    def get_edge_weight(self, node1, node2):
        matching_edges = filter(lambda edge: edge.target == node2,
                                self.node_edges(node1))
        return matching_edges[0].weight #todo this assumes only one edge from node1 to node2 exists

    def copy(self):
        return deepcopy(self)

    # DRAWING METHODS
    def draw(self, highlight_edges=None):
        nxg = nx.DiGraph()
        edges = [(e.source, e.target, {'weight':e.weight, 'inv_weight':1.0/e.weight}) for node_set in self._edges.values() for e in node_set]
        nxg.add_edges_from(edges)
        if len(self.node_positions) < len(self._nodes):
            # Calculate positions for nodes whose pos is not specified.
            pos = nx.spring_layout(nxg, weight='inv_weight', pos=self.node_positions, fixed=self.node_positions.keys() if self.node_positions else None)
        else:
            pos = self.node_positions

        f = plt.figure(figsize=(12,12))
        plt.gca().set_aspect('equal', adjustable='box')
        nx.draw_networkx_nodes(nxg, pos, node_color='w')
        nx.draw_networkx_edges(nxg, pos, edges)
        nx.draw_networkx_labels(nxg, pos)
        edge_labels=dict([((u,v,),"%s" % d['weight'])
                 for u,v,d in nxg.edges(data=True)])
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=edge_labels)


        if highlight_edges:
            nx.draw_networkx_edges(nxg, pos, highlight_edges, edge_color='r')

        plt.axis('off')
        plt.show()

    def draw_edges(self, edges):
        # print edges
        nx.draw_networkx_edges(nxg, pos, edges, edge_color='r')
        reduced_labels = {(u,v): edge_labels[(u,v)] for u,v,_ in edges}
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=reduced_labels, font_color='r')

        reduced_nodes = set([u for u,_,_ in edges])
        reduced_nodes.update([v for _,v,_ in edges])
        # nx.draw_networkx_nodes(nxg, pos, nodelist=reduced_nodes,  node_color='r')
        red_labels = {n:n for n in reduced_nodes}
        print red_labels
        nx.draw_networkx_labels(nxg, pos, labels=red_labels, font_color='r')

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

