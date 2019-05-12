#!/usr/bin/env python

import numpy as np
import rospy
import collections
from nav_msgs.msg import OccupancyGrid
from queue import PriorityQueue
from node import Node
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import time

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
        # Map in 2d
        self.map = None
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
            break

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
        Callback function for map receiving
        '''
        self.map = map_
        self.start = self.position(w=int(self.map.info.width*0.47), h=int(self.map.info.height*0.4))
        self.goal = self.position(w=int(self.map.info.width*0.52), h=int(self.map.info.height*0.5))
        self.graph = self.prepare_map(map_)
        self.start_node = self.graph[self.start.w][self.start.h]
        self.goal_node = self.graph[self.goal.w][self.goal.h]
        path = self.calculate_path()
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
            mk.lifetime = rospy.Duration(30)
            mk.color = self.color[type]
            self.marker_array.markers.append(mk)


if __name__ == '__main__':
    # Forward LPA* Search
    dsl = LPAstar()
    rospy.init_node('LPAstar', anonymous=True)
    path = rospy.Subscriber('/map', OccupancyGrid, dsl.process_map)
    rospy.spin()


