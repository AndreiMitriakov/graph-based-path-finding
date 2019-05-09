#!/usr/bin/env python

# https://answers.ros.org/question/10268/where-am-i-in-the-map/
# https://answers.ros.org/question/306110/where-is-the-origin-of-a-map-made-by-gmapping/

import sys
import numpy as np
import rospy
import collections
from nav_msgs.msg import OccupancyGrid
from queue import PriorityQueue
# g(s) - distance from the start node to the current node (sum cost through pred nodes)
# h(s, sa) - heuristic estimate of the distance between s and sa = c(s, sa) + h(sa, sgoal) for all sa - Succ(s)
# rhs(s) - one step lookahead based on g(s) = min sa (g(sa)+c(sa, s))

class Node:
    def __init__(self, w, h, prob, rhs, g, goal):
        self.goal = goal
        self.w = w
        self.h = h
        self.prob = prob
        self.rhs = rhs
        self.g = g
	self.pred= None

    def getHeuristic(self, goal = None):
	if goal == None:
	    goal = self.goal
        return round(((goal.w-self.w)**2+(goal.h-self.h)**2)**0.5, 3)

class DStarLite:

    def __init__(self):
        self.position = collections.namedtuple('Position', 'w h')
        # Finally, it should be defined in getter methods
        self.start = self.position(w=110, h=85)
        self.goal = self.position(w=100, h=120)
        self.queue = PriorityQueue()        

    def initialize(self):
        '''
        Implementation of method Initialize from pseudocode LPA*.
        '''
        self.graph[self.start.w][self.start.h].rhs = 0
        node_start = self.graph[self.start.w][self.start.h]
        priority = self.calculate_key(node_start)
	# predcessor is not defined because this is the start node
	node_start.pred = (np.nan, np.nan)
        self.queue.insert(node_start, priority)
        
    def calculate_key(self, node):
        return (min(node.g, node.rhs)+node.getHeuristic(), min(node.g, node.rhs))

    def calculate_path(self):
        '''
        Implementation of Main() from LPA*
        '''
        self.initialize()
	expansion = True
        while expansion:
            expansion = self.compute_shortest_path()


    def compute_shortest_path(self):
	goal_node = self.graph[self.goal.w][self.goal.h]
	while self.queue.get_top_key() < self.calculate_key(goal_node) or goal_node.rhs != goal_node.g:
	    node = self.queue.pop()
	    if node.g > node.rhs:
		node.g = node.rhs
	        for successor in node.get_successors():# to do
		    self.update_node(successor) # to do
	    else:
		node.g = np.inf
		self.update_node(node)
		for successor in node.get_successors():
		    update_node(successor)

    def update_node(self, node):
	pass

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
        self.graph = self.prepare_map(map_)
        path = self.calculate_path()
        return path

if __name__ == '__main__':
    # Forward LPA* Search
    dsl = DStarLite()
    rospy.init_node('DStarLite', anonymous=True)
    path = rospy.Subscriber('/map', OccupancyGrid, dsl.process_map)    
    rospy.spin()


