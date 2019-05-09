#!/usr/bin/env python

# https://answers.ros.org/question/10268/where-am-i-in-the-map/
# https://answers.ros.org/question/306110/where-is-the-origin-of-a-map-made-by-gmapping/

import sys
import numpy as np
import rospy
from namedlist import namedlist
from nav_msgs.msg import OccupancyGrid
# g(s) - distance from the start node to the current node (sum cost through pred nodes)
# h(s, sa) - heuristic estimate of the distance between s and sa = c(s, sa) + h(sa, sgoal) for all sa - Succ(s)
# rhs(s) - one step lookahead based on g(s) = min sa (g(sa)+c(sa, s))

class DStarLite:

    def __init__(self):
	self.position = namedlist('Position', 'w h')
	# Finally, it should be defined in getter methods
	self.start = self.position(w=110, h=85)
	self.goal = self.position(w=100, h=120)

    def get_map(self, map_):
        pass

    def get_start(self, start):
        pass

    def initialize(self, graph):
	'''
	Implementation of method Initialize from pseudocode LPA*.
	'''
	# Priority queue
        self.U = []
	# sq_map has a vector in every cell that contains probability, g, h, rhs, predcessor, successors
	# rhs and g are already inf // see prepare_map
	# rhs_start = 0
	graph[self.start.w][self.start.h].rhs = 0
	self.U.append(self.start, self.calculate_key(graph[self.start.w][self.start.h]))
	

    def calculate_key(self, graph, vert): # !!!!!
	return (min(cell.g, cell.rhs+self.), min())

    def calculate_path(self, graph):
	'''
	Implementation of Main() from LPA*
	'''
        self.initialize(graph)

    def prepare_map(self, map_):
	''' 
	map_.data is transformed into 2D array accordingly to size of map.
	Every element is a namedlist with attrs: prob g h rhs pred succ
	'''
	cell = namedlist('Vertex', 'prob g h rhs pred succ')
	sq_map = []
	i = 0
	for h in range(map_.info.height):
	    for w in range(map_.info.width):
		i += 1
		c = cell(prob=map_.data[w+h*map_.info.width-1], g=np.inf, h=np.nan, rhs=np.inf, pred=np.nan, succ=np.nan)
		sq_map.append(c)

	#https://stackoverflow.com/questions/53576138/writing-a-numpy-array-from-a-list-of-namedtuples
	sq_map = np.array(sq_map+[None])[:-1]
	sq_map = np.reshape(sq_map, (map_.info.width,  map_.info.height))
	return sq_map

    def get_map(self, map_):
	graph = self.prepare_map(map_)
	path = self.calculate_path(graph)
	return path

if __name__ == '__main__':
    # Forward LPA* Search
    dsl = DStarLite()
    rospy.init_node('DStarLite', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, dsl.get_map)    
    rospy.spin()


