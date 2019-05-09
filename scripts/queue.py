import numpy as np


class PriorityQueue:

    # Data structure for path finding search algorithms
    
    def __init__(self):
        self.queue = []

    def __str__(self):
	return str(self.queue)

    def get_top_key(self):
        '''
        returns the (numerically) lowest priority of any node in the queue (or infinity if the queue is empty)
        '''
	if self.queue == []:
	    return np.inf
	else:
	    return min([elem[1] for elem in self.queue])

	
    def pop(self):
        '''
        removes the node with the lowest priority from the queue and returns it
        '''
	index = [elem[1] for elem in self.queue].index(self.get_top_key())
        return self.queue.pop(index)

    def insert(self, node, priority):
        '''
        inserts a node with a given priority into the queue
        '''
	print 'insert node', node.w, node.h
        self.queue.append([node, priority])

    def remove(self, node):
        '''
        removes a node from the queue
        '''
        pass

    def contains(self, node):
        '''
        returns true if the queue contains the specified node, false if not
        '''
        pass
