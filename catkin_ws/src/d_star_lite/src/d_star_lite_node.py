#!/usr/bin/env python

from __future__ import division
import sys
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID

import math
import numpy as np
from d_star_lite.world import World
from d_star_lite.grid import *
from d_star_lite.queue import PriorityQueue
from d_star_lite.graph import Graph, get_intended_path
from d_star_lite.utils import *

inf = float("inf")


class DStarLiteNode():
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters:
        self.grid_resolution = self.setupParameter("~grid_resolution",0.5)
        self.occupancy_threshold = self.setupParameter("~occupancy_threshold",38.5)
        self.print_poses = self.setupParameter("~print_poses",False)
        self.set_viz_data = self.setupParameter("~set_viz_data",False)
        self.heuristic = euclidean_heuristic

        # State variables:
        self.grid = None
        self.old_grid = None
        self.graph = None
        self.old_graph = None
        self.graph_initialized = False
        self.graph_updated = False
        self.frame_id = None
        self.current_node = None
        self.current_point = None
        self.next_node = None
        self.path = None
        self.goal = None
        self.goal_idx = 0
        self.dispatched_end = False
        self.start = None
        self.last_start = None
        self.g = None
        self.rhs = None
        self.key_modifier = 0
        self.plan_in_progress = False
        self.map_displacement = None

        # Subscribers:
        self.sub_pose = rospy.Subscriber("~pose", PoseWithCovarianceStamped, self.updatePose, queue_size=1)
        self.sub_grid = rospy.Subscriber("~grid", OccupancyGrid, self.updateGraph, queue_size = 1)
        self.sub_goal = rospy.Subscriber("~goal_request", PoseStamped, self.updateGoal, queue_size=1)
        self.sub_goal_status = rospy.Subscriber("~goal_status", GoalStatusArray, self.checkGoal, queue_size=1)

        # Publishers:
        self.pub_goal = rospy.Publisher("~goal", MoveBaseActionGoal, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher("~path", Path, queue_size=1, latch=True)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updatePose(self, data):
        self.current_point = data.pose.pose.position
        if self.print_poses:
            rospy.loginfo("[%s] Received pose: (%s,%s)" %(self.node_name,self.current_point.x,self.current_point.y))
        if self.graph_initialized:
            self.current_node = self.resolve_point_to_node(self.current_point)
            if self.print_poses:
                rospy.loginfo("[%s] Resolved pose to node: %s" %(self.node_name,self.current_node))
            """
            if self.plan_in_progress:
                if self.current_node == self.goal:
                    rospy.loginfo("[%s] Reached goal: %s" %(self.node_name,self.current_node))
                    self.plan_in_progress = False
                elif self.current_node == self.next_node:
                    rospy.loginfo("[%s] Reached node %s, performing next D* Lite iteration." %(self.node_name,self.next_node))
                    self.iterateDStarLite()
            """            

    def checkGoal(self,data):
        if self.plan_in_progress:
            for status in data.status_list:
                if status.goal_id.id == str(self.goal_idx):
                    if status.status == status.SUCCEEDED:
                        self.goal_idx += 1
                        if self.dispatched_end:
                            rospy.loginfo("[%s] Reached goal: %s" %(self.node_name,self.current_node))
                            self.plan_in_progress = False
                        else:
                            rospy.loginfo("[%s] Reached node %s, performing next D* Lite iteration." %(self.node_name,self.next_node))
                            self.iterateDStarLite()     

    def updateGraph(self, data):
        # callback function for map update, should produce self.graph
        self.frame_id = data.header.frame_id
        rospy.loginfo("[%s] Received new grid, shape: (%s,%s), resolution: %s, frame_id: %s" %(self.node_name,data.info.width,data.info.height,data.info.resolution, self.frame_id))
        rospy.loginfo("[%s] Map translation: (%s,%s)" %(self.node_name,data.info.origin.position.x,data.info.origin.position.y))
        self.map_displacement = data.info.origin.position

        if self.graph_initialized:
            rospy.loginfo("[%s] Saving old grid for comparison..." %(self.node_name))
            start_time = rospy.get_time()
            self.old_grid = self.grid.copy()
            #self.old_graph = self.graph.copy()
            total_time = rospy.get_time() - start_time
            rospy.loginfo("[%s] Old grid saved (%s s)." %(self.node_name,total_time))

        self.grid = self.downSample(data.data,data.info.height,data.info.width,data.info.resolution)
        rospy.loginfo("[%s] Downsampled grid, new shape: %s, new resolution: %s" %(self.node_name,self.grid.shape,self.grid_resolution))

        if not self.graph_initialized:
            self.old_grid = self.grid.copy()
            rospy.loginfo("[%s] Converting new grid to graph..." %(self.node_name))
            start_time = rospy.get_time()
            self.graph = Graph.fromArray(self.grid.astype(int), set_viz_data=self.set_viz_data)
            total_time = rospy.get_time() - start_time
            rospy.loginfo("[%s] Created graph (%s s), total nodes: %s" %(self.node_name,total_time,len(self.graph.get_all_nodes())))

        if self.graph_initialized:
            if self.plan_in_progress:
                self.graph_updated = True
            else:
                self.get_changed_edges()
        else:
            self.graph_initialized = True
        if self.current_point:
            self.current_node = self.resolve_point_to_node(self.current_point)
            rospy.loginfo("[%s] Resolved pose to node: %s" %(self.node_name,self.current_node))

    def downSample(self, grid, height, width, resolution):
        new_width = int(round((width)*resolution/self.grid_resolution));
        new_height = int(round((height)*resolution/self.grid_resolution));
        new_grid = np.zeros((new_width,new_height))
        for index_x in range(new_width):
            for index_y in range(new_height):

                original_map_x = int(round(index_x *self.grid_resolution/resolution))
                if original_map_x > width:
                    original_map_x = width
                
                original_map_y = int(round(index_y *self.grid_resolution/resolution))
                if original_map_y > height:
                    original_map_y = height
                
                if grid[original_map_x*width+original_map_y] > self.occupancy_threshold:
                    new_grid[index_x][index_y] = 1
        return new_grid
        

    def updateGoal(self, data):
        self.dispatched_end = False
        goal_point = data.pose.position
        rospy.loginfo("[%s] Received goal point: (%s,%s)" %(self.node_name,goal_point.x,goal_point.y))        
        if self.graph_initialized:
            self.goal = self.resolve_point_to_node(goal_point)
            rospy.loginfo("[%s] Resolved goal to node: %s" %(self.node_name,self.goal))
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
        self.next_node = self.current_node
        self.last_start = self.start

        # Set g=inf and rhs=inf for all nodes, except the goal node, which has rhs=0
        self.g = {node:inf for node in self.graph.get_all_nodes()}
        self.rhs = {node:inf for node in self.graph.get_all_nodes()}
        self.rhs[self.goal] = 0

        # Set the key modifier k_m to 0
        self.key_modifier = 0

        # Initialize the queue using the priority function calc_key
        self.queue = PriorityQueue(f=lambda node: self.calc_key(node))
        self.queue.insert(self.goal)

        # Process any pending graph updates:
        if self.graph_updated:
            self.get_changed_edges()

    def iterateDStarLite(self):
        self.updateKeyModifier()
        if self.graph_updated:
            self.checkEdgesDStarLite()
            self.graph_updated = False
        self.getPathDStarLite()

    def getPathDStarLite(self):
        rospy.loginfo("[%s] Computing shortest path..." %(self.node_name))
        start_time = rospy.get_time()
        self.compute_shortest_path()
        total_time = rospy.get_time() - start_time

        if self.g[self.start] == inf:
            self.plan_in_progress = False
            rospy.loginfo("[%s] No feasible path." %(self.node_name))
            return

        rospy.loginfo("[%s] Computed shortest path (%s s), extracting next node..." %(self.node_name, total_time))        
        start_time = rospy.get_time()
        self.start = min(self.graph.get_successors(self.next_node),
                    key = lambda neighbor: (self.graph.get_edge_weight(self.next_node, neighbor)
                                            + self.g[neighbor]))
        self.path = self.get_path()
        next_idx = int(1.0/(2*self.grid_resolution))
        if next_idx > (len(self.path) - 1):
            next_idx = len(self.path) - 1
        self.next_node = self.path[next_idx]

        total_time = rospy.get_time() - start_time
        rospy.loginfo("[%s] Extracted next node (%s s): %s" %(self.node_name, total_time, self.next_node))
        rospy.loginfo("[%s] Total nodes in path: %s" %(self.node_name, len(self.path)))
        self.publishNextPoint()
        self.publishPath()

    def updateKeyModifier(self):
        self.key_modifier = self.key_modifier + self.heuristic(self.last_start, self.start)
        self.last_start = self.start

    def checkEdgesDStarLite(self):
        rospy.loginfo("[%s] Checking for edge changes..." %(self.node_name))
        start_time = rospy.get_time()
        #changed_edges = self.old_graph.get_changed_edges(self.graph)
        changed_edges = self.get_changed_edges()
        total_time = rospy.get_time() - start_time
        if changed_edges:
            rospy.loginfo("[%s] Edge changes detected (%s s): %s, performing updates..." %(self.node_name,total_time,len(changed_edges)))
            start_time = rospy.get_time()
            #for (old_edge, new_edge) in changed_edges:
            for edge in changed_edges:
                self.update_vertex(edge.source)
                #if old_edge and new_edge: #edge simply changed weight
                #    self.update_vertex(old_edge.source)
                #elif not old_edge: #new edge was added
                #    raise NotImplementedError("Edge addition not yet supported")
                #else: #old edge was deleted
                #    raise NotImplementedError("Edge deletion not yet supported")
            total_time = rospy.get_time() - start_time
            rospy.loginfo("[%s] Updates complete (%s s)." %(self.node_name,total_time))
        else:
            rospy.loginfo("[%s] No edge changes detected (%s s)." %(self.node_name,total_time))
        self.graph_updated = False

    def get_changed_edges(self):
        diff_grid = self.grid - self.old_grid
        new_obstacles = zip(*np.where(diff_grid == 1))
        new_free = zip(*np.where(diff_grid == -1))

        updated_edges = []

        for n in new_obstacles:
            node = (n[1],n[0])
            neighbors = self.graph.get_successors(node)
            for neighbor in neighbors:
                updated_edges += [self.graph.set_edge_weight(node,neighbor,inf)]
                updated_edges += [self.graph.set_edge_weight(neighbor,node,inf)]

        for n in new_free:
            node = (n[1],n[0])
            neighbors = self.graph.get_successors(node)
            for neighbor in neighbors:
                if self.grid[neighbor[1],neighbor[0]] == 0:
                    updated_edges += [self.graph.set_edge_weight(node,neighbor,self.weighting(node,neighbor))]
                    updated_edges += [self.graph.set_edge_weight(neighbor,node,self.weighting(neighbor,node))]
        
        return updated_edges

    def publishPath(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        poses = []
        for node in self.path:
            x,y = self.convert_node_to_point(node)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            poses += [pose_msg]
        msg.poses = poses
        self.pub_path.publish(msg)

    def publishNextPoint(self):
        x_next, y_next = self.convert_node_to_point(self.next_node)
        w_next = self.computeNextOrientation(x_next,y_next)
        rospy.loginfo("[%s] Converted next node to point: (%s,%s), orientation: %s" %(self.node_name, x_next, y_next, w_next))

        goal_id = GoalID()
        goal_id.id = str(self.goal_idx)

        msg = MoveBaseActionGoal() 
        msg.goal_id = goal_id
        msg.goal.target_pose.header.frame_id = self.frame_id
        msg.goal.target_pose.pose.position.x = x_next
        msg.goal.target_pose.pose.position.y = y_next
        msg.goal.target_pose.pose.position.z = 0
        msg.goal.target_pose.pose.orientation.x = 0
        msg.goal.target_pose.pose.orientation.y = 0
        msg.goal.target_pose.pose.orientation.z = math.sin(w_next/2)
        msg.goal.target_pose.pose.orientation.w = math.cos(w_next/2)
        self.pub_goal.publish(msg)
        rospy.loginfo("[%s] Dispatched goal point: (%s,%s)" %(self.node_name,x_next,y_next))

        if self.next_node == self.goal:
            self.dispatched_end = True

    def computeNextOrientation(self,x,y):
        dx = x - self.current_point.x
        dy = y - self.current_point.y
        return math.atan2(dy,dx)

    def check_ready(self):
        if not self.graph_initialized:
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

    def weighting(self, node1, node2):
        return self.heuristic(node1,node2)

    def get_path(self):
        return get_intended_path(self.start, self.goal, self.graph, self.g)

    def calc_key(self, node):
        return calc_key_helper(node, self.g, self.rhs, self.start, self.key_modifier, heuristic=self.heuristic)

    def update_vertex(self, node):
        return update_vertex_helper(node, self.g, self.rhs, self.goal, self.graph, self.queue)

    def compute_shortest_path(self):
        return compute_shortest_path_helper(self.g, self.rhs, self.start, self.goal, self.key_modifier, self.graph, self.queue, self.heuristic)

    def resolve_point_to_node(self, point):
        return resolve_point_to_node_helper(point, self.graph, self.grid_resolution,self.map_displacement)

    def convert_node_to_point(self, node):
        return convert_node_to_point_helper(node, self.grid_resolution,self.map_displacement)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == "__main__":
    rospy.init_node('d_star_lite_node')
    d_star_lite_node = DStarLiteNode()
    rospy.on_shutdown(d_star_lite_node.onShutdown)
    rospy.spin()
