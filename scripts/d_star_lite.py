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
        self.succ_set = False
        self.rhs = rhs
        self.g = g
        self.pred= []
        self.priority = None
        self.succ = []
        self.steps = ((-1, 0), (1, 0), (0, -1), (0, 1))

    def __eq__(self, node): 
        if not isinstance(node, Node):
            # don't attempt to compare against unrelated types
            return NotImplemented
        return self.w == node.w and self.h == node.h

    def get_heuristic(self):
        return round(((self.goal.w-self.w)**2+(self.goal.h-self.h)**2)**0.5, 3)

    def get_successors(self, graph):
        ''' 
        Set successors for a node.
        Return successor nodes.
        Append the current node as a parent to successor nodes.
        '''
        if self.succ_set == False:
            for step in self.steps:
                node = graph[self.w + step[0]][self.h + step[1]]
                if node not in self.pred:
                    self.succ.append(node)
                    # Add self node as the predecessor to the current successor node    
                    node.pred.append(self)
            self.succ_set = True
            return self.succ
        else:   
            return self.succ
            
    def get_predecessors(self):
        return self.pred

    def get_cost_to(self, node):
        '''
        Called in context predecessor.getCostTo(node).
        Estimates cost from the predecessor node (self) to the current one (node).
        It is acceptable to say that the cost is 0 between two node.

        '''
        # 1.0 distance in pixels travelled to a close node
        return 1.0

class LPAstar:

    def __init__(self):
        self.position = collections.namedtuple('Position', 'w h')
        # Finally, it should be defined in getter methods
        self.start = self.position(w=110, h=85)
        self.goal = self.position(w=100, h=120)
        self.queue = PriorityQueue()
        self.directed_graph = []    
        # Map in undirected graph form, defined latter
        self.graph = None    
        # Map in 2d 
        self.map = None
        
    def initialize(self):
        '''
        Implementation of method Initialize from pseudocode LPA*.
        '''
        self.graph[self.start.w][self.start.h].rhs = 0
        node_start = self.graph[self.start.w][self.start.h]
        priority = self.calculate_key(node_start)
        # No need to set predecessor     
        self.queue.insert(node_start, priority)

    def calculate_key(self, node):
        return (min(node.g, node.rhs)+node.get_heuristic(), min(node.g, node.rhs))

    def get_path(self):
        '''
        Return path to the goal node.
        It starts from the goal node going by predecessors with the smallest cost value.
        '''
        pass

    def calculate_path(self):
        '''
        Implementation of Main() from LPA* 
        '''
        self.initialize()
        while True:
            self.compute_shortest_path()
            self.get_path()
            break

    def compute_shortest_path(self):
        goal_node = self.graph[self.goal.w][self.goal.h]
        while self.queue.get_top_key() < self.calculate_key(goal_node) or goal_node.rhs != goal_node.g:
            node = self.queue.pop()
            print('Current node', node.w, node.h)
            if node == False:
                print('No nodes in the priority queue')
                break
            if node.g > node.rhs:
                node.g = node.rhs
                for successor in node.get_successors(self.graph):
                    self.update_node(successor) 
            else:
                node.g = np.inf
                self.update_node(node)
                for successor in node.get_successors(self.graph):
                    self.update_node(successor)


    def update_node(self, node):
        '''
        Recalculates rhs for a node and removes it from the queue.
        If the node has become locally inconsistent, 
        it is (re-)inserted into the queue with its new key.
        '''
        node.rhs = np.inf
        # PROBLEM IS THAT THE PREDECESSOR HAS INF G VALUE !!!
        for predecessor in node.get_predecessors():
            node.rhs = min(node.rhs, predecessor.g + predecessor.get_cost_to(node))
        print('min', node.rhs, predecessor.g, predecessor.get_cost_to(node))
        # Next condition can be omitted
        if self.queue.contains(node):
            self.queue.remove(node) 
        print('node.g != node.rhs', node.g, node.rhs)
        if node.g != node.rhs:
            self.queue.insert(node, self.calculate_key(node))

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
    dsl = LPAstar()
    rospy.init_node('LPAstar', anonymous=True)
    path = rospy.Subscriber('/map', OccupancyGrid, dsl.process_map)    
    rospy.spin()


