#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import collections
import copy
from nav_msgs.msg import OccupancyGrid
from queue import PriorityQueue
from node import Node
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
        # Finally, it should be defined in getter methods
        self.start = None
        self.goal = None
        self.queue = PriorityQueue()
        self.directed_graph = []    
        # Map in undirected graph form, defined latter
        self.graph = None
        self.pnts = 0
        self.nmb_received_maps = 0
        self.map = OccupancyGrid()
        self.map_prev = OccupancyGrid()
        self.path = []
        self.marker_array = MarkerArray()
        self.pub_markers = rospy.Publisher('/markers', MarkerArray, queue_size=10)
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
        nds = []
        node = self.goal_node
        while node != self.start_node:
            prob_nodes = sorted(node.get_predecessors(), key=lambda node_: node.get_cost_to(node_) + node_.g)
            nds.append(node)
            if len(prob_nodes) != 0:
                node = prob_nodes[0]
                self.path.append(node)
                self.change_color(node, 'start')
            else:
                break
        print nds
        self.change_color(self.start_node, 'start')
        self.change_color(self.goal_node, 'goal')
        self.pub_markers.publish(self.marker_array)

    def calculate_path(self):
        '''
        Implementation of Main() from LPA*
        '''
        self.initialize()
        i = 0
        while True:
            self.compute_shortest_path()
            path = self.get_path()
            # In future, update_map has to be called only after new map receiving
            # A while loop should be implemented exiting from itself when dsl.process_map callback called
            # while self.nothing_new:
            #   time.sleep(1)
            # self.nothing_new = True # self.nothing_new = False in dsl.process_map()
            while self.map_prev.header.stamp.secs == self.map.header.stamp.secs:
                time.sleep(1)
            changed_nodes = self.update_map()
            # Changed nodes consist of nodes around appeared obstacle.
            # Nodes located in the place of new obstacle are dropped from markers and not updated
            # If obstacle disappears map is updated nodes around obstacle are also added to be updated.
            for node in changed_nodes:
                self.update_node(node, type='update')
            i += 1
            self.map_prev = copy.deepcopy(self.map)

    def update_node(self, node, type = 'nowhere'):
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

    def update_map(self):
        '''
        Compares two maps in order to find changed traversal costs.
        :param map_new: new map
        :param map_old: old map
        :return: list of changed nodes
        '''
        changed_nodes = [(i, prob) for i, prob in enumerate(self.map.data) if self.map_prev.data[i] != self.map.data[i]]
        nodes = []
        contains = lambda marker, w, h: marker.pose.position.x == w * self.map.info.resolution and \
                                        marker.pose.position.y == h * self.map.info.resolution
        for index, prob in changed_nodes:
            h = int(index / self.map.info.width)
            w = index - h * self.map.info.width
            node = self.graph[w][h]
            # Update graph
            node.prob = prob
            # Pop all touched nodes from marker list
            for i, marker in enumerate(self.marker_array.markers):
                if contains(marker, w, h):
                    marker.color = self.color['goal']
            # Form a list of nodes to be updated, these are nodes around obstacle
            if node not in nodes:
                nodes.append(node)
                for succ in node.get_successors(self.graph):
                    if succ not in nodes:
                        nodes.append(succ)
                for pred in node.get_predecessors():
                    if pred not in nodes:
                        nodes.append(pred)
        return nodes

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
                    self.update_node(successor)
            else:
                node.g = np.inf
                self.update_node(node)
                for successor in node.get_successors(self.graph):
                    self.update_node(successor)
            # self.pub_markers.publish(self.marker_array)
            self.change_color(node, 'usual')

    def change_color(self, node, clr):
        for marker in self.marker_array.markers:
            if node.id == marker.id:
                marker.color = self.color[clr]


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
        if self.nmb_received_maps == 0:
            self.map = map_
            self.map_prev = copy.deepcopy(map_)
            self.start = self.position(w=int(self.map.info.width * 0.31), h=int(self.map.info.height * 0.49))
            self.goal = self.position(w=int(self.map.info.width * 0.46), h=int(self.map.info.height * 0.49))
            self.graph = self.prepare_map(map_)
            self.start_node = self.graph[self.start.w][self.start.h]
            self.goal_node = self.graph[self.goal.w][self.goal.h]
            self.calculate_path()
        else:
            self.map = map_
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
    '''Forward LPA* Search'''
    dsl = LPAstar()
    rospy.init_node('LPAstar', anonymous=True)
    path = rospy.Subscriber('/map', OccupancyGrid, dsl.process_map)
rospy.spin()