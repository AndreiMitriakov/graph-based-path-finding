#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import collections
import copy
from nav_msgs.msg import OccupancyGrid
from queue import PriorityQueue
from node import Node
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import time
from nav_msgs.srv import GetMap

'''
# g(s) - distance from the start node to the current node (sum cost through pred nodes)
# h(s, sa) - heuristic estimate of the distance between s and sa = c(s, sa) + h(sa, sgoal) for all sa - Succ(s)
# rhs(s) - one step lookahead based on g(s) = min sa (g(sa)+c(sa, s))
'''


class LPAstar:

    def __init__(self):
        self.position = collections.namedtuple('Position', 'w h')
        self.obstacle = collections.namedtuple('Obstacle', 'w h height')
        # Finally, it should be defined in getter methods
        self.start = None
        self.goal = None
        self.obst = None
        self.map_update = OccupancyGridUpdate()
        self.queue = PriorityQueue()
        self.directed_graph = []    
        # Map in undirected graph form, defined latter
        self.graph = None
        self.pnts = 0
        # Map in 2d
        self.map = None
        self.new_map = None
        self.path = []
        self.marker_array = MarkerArray()
        self.pub_markers = rospy.Publisher('/markers', MarkerArray, queue_size=10)
        self.pub_map_updates = rospy.Publisher('/map_updates', OccupancyGridUpdate, queue_size=10)
        self.mid = 0
        self.color = {'start': ColorRGBA(0, 1, 0, 1), 'goal': ColorRGBA(1, 0, 0, 1), 'usual': ColorRGBA(0, 0, 1, 1),
                      'current': ColorRGBA(0.5, 0, 0.5, 1), 'path': ColorRGBA(0, 0.5, 0.5, 1)}

    def initialize(self):
        '''
        Implementation of method Initialize from pseudocode LPA*.
        '''
        self.graph[self.start.w][self.start.h].rhs = 0
        node_start = self.graph[self.start.w][self.start.h]
        # adding start and goals nodes to visualization marker list
        priority = self.calculate_key(node_start)
        # No need to set predecessor     
        self.queue.insert(node_start, priority)
        self.add_marker(self.graph[self.start.w][self.start.h], 'start')
        self.add_marker(self.graph[self.goal.w][self.goal.h], 'goal')

    def calculate_key(self, node):
        return (min(node.g, node.rhs)+node.get_heuristic(), min(node.g, node.rhs))

    def get_path(self):
        '''
        Return path to the goal node.
        It starts from the goal node going by predecessors with the smallest cost value.
        '''
        node = self.goal_node
        while node != self.start_node:
            prob_nodes = sorted(node.get_predecessors(), key=lambda node_: node.get_cost_to(node_) + node_.g)
            node = prob_nodes[0]
            self.path.append(node)
            self.change_color(node, 'path')
        self.change_color(self.start_node, 'start')
        self.change_color(self.goal_node, 'goal')
        self.pub_markers.publish(self.marker_array)

    def calculate_path(self):
        '''
        Implementation of Main() from LPA* 
        '''
        self.initialize()
        while True:
            self.compute_shortest_path()
            path = self.get_path()
            time.sleep(1)
            self.add_obstacle()
            self.redefine_costs()
            break

    def redefine_costs(self):
        '''

        '''
        xo = min(self.obst1.w, self.obst2.w)
        yo = min(self.obst1.h, self.obst2.h)
        width = abs(self.obst1.w - self.obst2.w)
        height = abs(self.obst1.h - self.obst2.h)
        markers_id = [mrk.id for mrk in self.marker_array.markers]
        # Find markers that are not
        for i in range(width * height):
            self.graph[xo+width][yo+height].prob = 100
            for marker_index, marker in enumerate(self.marker_array.markers):
                if self.graph[xo+width][yo+height].id == marker.id:
                    self.marker_array.markers[marker_index] = 2
        self.pub_markers.publish(self.marker_array)
        # Define touched nodes needed to be updated
        self.nodes_to_update = []
        for i in range(height + 2):
            node_left = self.graph[xo-1][yo + i - 1]
            node_right = self.graph[xo + width + 1][yo + i - 1]
            if node_left.id in markers_id:
                self.nodes_to_update.append(node_left)
            if node_right.id in markers_id:
                self.nodes_to_update.append(node_right)
        for i in range(width):
            node_bot = self.graph[xo + i][yo]
            node_top = self.graph[xo + i][yo + height]
            if node_bot.id in markers_id:
                self.nodes_to_update.append(node_bot)
            if node_top.id in markers_id:
                self.nodes_to_update.append(node_top)

    def add_obstacle(self):
        '''
        Calculates obstacle line based on two points self.obst1 and self.obst2.
        '''
        self.map_update.header.seq = 0
        self.map_update.header.stamp = rospy.Time.now()
        self.map_update.header.frame_id = self.map.header.frame_id
        self.map_update.x = self.obst.w
        self.map_update.y = self.obst.h
        self.map_update.width = 1
        self.map_update.height = self.obst.height
        self.map_update.data = []
        for x in range(self.map_update.width*self.map_update.height):
            self.map_update.data.append(100)
        self.map_update.data = tuple(self.map_update.data)
        self.pub_map_updates.publish(self.map_update)
        self.map = rospy.ServiceProxy('static_map', GetMap)().map


    def compute_shortest_path(self):
        goal_node = self.graph[self.goal.w][self.goal.h]
        cnt = 0
        while self.queue.get_top_key() < self.calculate_key(goal_node) or goal_node.rhs != goal_node.g:
            cnt += 1
            node = self.queue.pop()
            self.change_color(node, 'current')
            if node.g > node.rhs:
                node.g = node.rhs # Start 0, 0
                for successor in node.get_successors(self.graph):
                    self.update_node(successor, cnt)
            else:
                node.g = np.inf
                self.update_node(node, cnt)
                for successor in node.get_successors(self.graph):
                    self.update_node(successor, cnt)
            self.pub_markers.publish(self.marker_array)
            self.change_color(node, 'usual')

    def change_color(self, node, clr):
        for marker in self.marker_array.markers:
            if node.id == marker.id:
                marker.color = self.color[clr]

    def update_node(self, node, cnt):
        '''
        Recalculates rhs for a node and removes it from the queue.
        If the node has become locally inconsistent, 
        it is (re-)inserted into the queue with its new key.
        Updating not start node.
        '''
        if node.w != self.start.w or node.h != self.start.h:
            node.rhs = np.inf
            for predecessor in node.get_predecessors():
                node.rhs = min(node.rhs, predecessor.g + predecessor.get_cost_to(node))
            if self.queue.contains(node):
                self.queue.remove(node)
            if node.g != node.rhs:
                self.queue.insert(node, self.calculate_key(node))
                self.add_marker(node)

    def prepare_map(self, map_):
        ''' 
        map_.data is transformed into 2D array accordingly to size of map.
        Every element is a namedlist with attrs: prob g h rhs pred succ
        '''
        # node = Node('Node', 'prob g h rhs pred succ')
        graph = []
        i = 0
        for w in range(map_.info.width):
            line= []
            for h in range(map_.info.height):
                i += 1
                node = Node(prob=map_.data[w+h*map_.info.width-1], g=np.inf, rhs=np.inf, w=w, h=h, goal=self.goal)
                line.append(node)
            graph.append(line)
        graph = np.array(graph)
        # graph = np.reshape(graph, (map_.info.width,  map_.info.height))
        return graph

    def process_map(self, map_):
        '''
        Callback function for map published message
        '''
        self.map = map_
        self.start = self.position(w=int(self.map.info.width * 0.34), h=int(self.map.info.height * 0.43))
        self.goal = self.position(w=int(self.map.info.width * 0.5), h=int(self.map.info.height * 0.43))
        self.obst = self.obstacle(w=int(self.map.info.width * 0.42), h=int(self.map.info.height * 0.43),
                                  height=int(self.map.info.height * 0.07))
        self.graph = self.prepare_map(map_)
        self.start_node = self.graph[self.start.w][self.start.h]
        self.goal_node = self.graph[self.goal.w][self.goal.h]
        self.calculate_path()
        path = None
        return path

    def add_marker(self, node, type='usual'):
        inside = False
        for marker in self.marker_array.markers:
            if marker.id == node.id:
                inside = True
                break
        if inside == False:
            mk = Marker()
            mk.header.seq = 0
            mk.header.stamp = rospy.Time.now()
            mk.header.frame_id = self.map.header.frame_id
            mk.id = node.id
            mk.type = 2
            mk.action = 0
            mk.pose.position.x = node.w * self.map.info.resolution
            mk.pose.position.y = node.h * self.map.info.resolution
            mk.pose.position.z = 0
            mk.pose.orientation.x = 0
            mk.pose.orientation.y = 0
            mk.pose.orientation.z = 0
            mk.pose.orientation.w = 1
            mk.scale = Vector3(self.map.info.resolution * 2, self.map.info.resolution * 2, self.map.info.resolution * 2)
            mk.lifetime = rospy.Duration(5)
            mk.color = self.color[type]
            self.marker_array.markers.append(mk)


if __name__ == '__main__':
    # Forward LPA* Search
    dsl = LPAstar()
    rospy.init_node('LPAstar', anonymous=True)
    path = rospy.Subscriber('/map', OccupancyGrid, dsl.process_map)
    rospy.spin()


