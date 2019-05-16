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
            return self.queue[0].priority

    def print_queue(self):
        print('Nodes in queue')
        for el in self.queue:
            print(el.id, el.priority)

    def pop(self):
        '''
        removes the node with the lowest priority from the queue and returns it
        '''
        if len(self.queue) == 0:
            return False
        else:
            return self.queue.pop(0)

    def _sort(self):
        self.queue = sorted(self.queue, key=lambda node: (node.priority[0], node.priority[1]))

    def insert(self, node, priority):
        '''
        inserts a node with a given priority into the queue
        '''
        node.priority = priority
        self.queue.append(node)
        self._sort()

    def remove(self, node):
        '''
        removes a node from the queue
        '''
        # To improve
        for i, node_q in enumerate(self.queue):
            if node == node_q:
                self.queue.pop(i)
                break

    def contains(self, node):
        '''
        returns true if the queue contains the specified node, false if not
        '''
        if node in self.queue:
            return True
        else:
            return False