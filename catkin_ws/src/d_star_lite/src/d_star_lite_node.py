#!/usr/bin/env python

from __future__ import division
import sys
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal

import math
import numpy as np
from d_star_lite.world import World
from d_star_lite.grid import *
from d_star_lite.queue import PriorityQueue
from d_star_lite.graph import get_intended_path
from d_star_lite.utils import *

inf = float("inf")


class DStarLiteNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters:
        self.grid_resolution = self.setupParameter("~grid_resolution",0.2)
        self.occupancy_threshold = self.setupParameter("~occupancy_threshold",0.1)
        self.heuristic = grid_heuristic

        # State variables:
        self.graph = None
        self.old_graph = None
        self.current_node = None
        self.nex_node = None
        self.goal = None
        self.start = None
        self.last_start = None
        self.g = None
        self.rhs = None
        self.key_modifier = 0
        self.plan_in_progress = False

        # Subscribers:
        self.sub_pose = rospy.Subscriber("~pose", PoseWithCovarianceStamped, self.updatePose, queue_size=1)
        self.sub_grid = rospy.Subscriber("~grid", OccupancyGrid, self.updateGraph, queue_size = 1)
        self.sub_goal = rospy.Subscriber("~goal_request", PoseStamped, self.updateGoal, queue_size=1)

        # Publishers:
        self.pub_goal = rospy.Publisher("~goal", MoveBaseActionGoal, queue_size=1, latch=True)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updatePose(self, data):
        current_point = data.pose.pose.position
        self.current_node = self.resolve_point_to_node(current_point)
        if (self.current_node == self.goal):
            self.plan_in_progress = False
        elif self.plan_in_progress and self.current_node == self.next_node:
            self.iterateDStarLite()

    def updateGraph(self, data):
        # callback function for map update, should produce self.graph
        new_width = int(round((data.info.width)*data.info.resolution/self.grid_resolution));
        new_height = int(round((data.info.height)*data.info.resolution/self.grid_resolution));
        new_grid = np.zeros((new_width,new_height))
        for index_x in range(new_width):
            for index_y in range(new_height):

                original_map_x = int(round(index_x *self.grid_resolution/data.info.resolution))
                if original_map_x > data.info.width:
                    original_map_x = data.info.width
                
                original_map_y = int(round(index_y *self.grid_resolution/data.info.resolution))
                if original_map_y > data.info.height:
                    original_map_y = data.info.height
                
                if data.data[original_map_x*data.info.width+original_map_y] > self.occupancy_threshold:
                    new_grid[index_x][index_y] = 1
        self.grid = new_grid


    def updateGoal(self, data):
        goal_point = data.pose.position
        self.goal = self.resolve_point_to_node(goal_point)
        if self.check_ready():
            rospy.loginfo("[%s] Starting D-star Lite plan." %(self.node_name))
            self.startDStarLite()

    def startDStarLite(self):
        self.plan_in_progress = True
        self.initializeDStarLite()
        self.getPathDStarLite()

    def initializeDStarLite(self):
        # Set start to current position
        self.start = self.current_node
        self.last_start = self.start

        # Set g=inf and rhs=inf for all nodes, except the goal node, which has rhs=0
        self.g = {node:inf for node in self.graph.get_all_nodes()}
        self.rhs = {node:inf for node in self.graph.get_all_nodes()}
        self.rhs[self.goal] = 0

        # Set the key modifier k_m to 0
        self.key_modifier = 0

        # Initialize the queue using the priority function calc_key
        self.queue = PriorityQueue(f=lambda node: self.calc_key(node))
        self.queue.insert(goal)

    def iterateDStarLite(self):
        self.checkEdgesDStarLite()
        self.getPathDStarLite()

    def getPathDStarLite(self):
        self.compute_shortest_path()
        if self.g[start] == inf:
            self.plan_in_progress = False
            rospy.loginfo("[%s] No feasible path." %(self.node_name))
            return
        self.start = min(self.graph.get_successors(self.start),
                    key = lambda neighbor: (self.graph.get_edge_weight(self.start, neighbor)
                                            + self.g[neighbor]))
        self.old_graph = self.graph.copy()
        intended_path = self.get_path()
        self.next_node = intended_path[0]
        self.publishNextPoint()

    def checkEdgesDStarLite(self):
        changed_edges = self.old_graph.get_changed_edges(self.graph)
        if changed_edges:
            self.key_modifier = self.key_modifier + self.heuristic(self.last_start, self.start)
            self.last_start = self.start
            for (old_edge, new_edge) in changed_edges:
                if old_edge and new_edge: #edge simply changed weight
                    self.update_vertex(old_edge.source)
                elif not old_edge: #new edge was added
                    raise NotImplementedError("Edge addition not yet supported")
                else: #old edge was deleted
                    raise NotImplementedError("Edge deletion not yet supported")

    def publishNextPoint(self, node):
        msg = MoveBaseActionGoal() 
        msg.goal.target_pose.pose.point.x = self.next_node[0]
        msg.goal.target_pose.pose.point.y = self.next_node[1]
        self.pub_goal.publish(msg)
        rospy.loginfo("[%s] Dispatched goal point: %s" %(self.node_name, node))

    def check_ready(self):
        if self.graph == None:
            rospy.loginfo("[%s] Not ready for planning: no graph." %(self.node_name))
            return False
        if self.current_node == None:
            rospy.loginfo("[%s] Not ready for planning: not localized." %(self.node_name))
            return False
        if self.goal == None:
            rospy.loginfo("[%s] Not ready for planning: did not receive goal." %(self.node_name))
            return False
        else:
            return True

    def get_path(self):
        return get_intended_path(self.start, self.goal, self.graph, self.g)

    def calc_key(self, node):
        return calc_key_helper(node, self.g, self.rhs, self.start, self.key_modifier, heuristic=self.heuristic)

    def update_vertex(self, node):
        return update_vertex_helper(node, self.g, self.rhs, self.goal, self.graph, self.queue)

    def compute_shortest_path(self):
        return compute_shortest_path_helper(self.g, self.rhs, self.start, self.goal, self.key_modifier, self.graph, self.queue)

    def resolve_point_to_node(self, point):
        return resolve_point_to_node_helper(point, self.graph)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == "__main__":
    rospy.init_node('d_star_lite_node')
    d_star_lite_node = DStarLiteNode()
    rospy.on_shutdown(d_star_lite_node.onShutdown)
    rospy.spin()
