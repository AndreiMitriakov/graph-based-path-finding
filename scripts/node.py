import numpy as np

class Node:
    def __init__(self, w, h, prob, rhs, g, goal):
        self.goal = goal
        zero_line = {1: '00', 2: '0', 3: ''}
        self.id = int(str(w) + zero_line[len(str(h))] + str(h))
        self.w = w
        self.h = h
        self.prob = prob
        self.succ_set = False
        self.rhs = rhs
        self.g = g
        self.pred = []
        self.priority = None
        self.succ = []
        self.steps = ((-1, 0), (1, 0), (0, -1), (0, 1))

    def __eq__(self, node):
        if not isinstance(node, Node):
            # don't attempt to compare against unrelated types
            return NotImplemented
        return self.w == node.w and self.h == node.h

    def get_heuristic(self):
        return round(((self.goal.w - self.w) ** 2 + (self.goal.h - self.h) ** 2) ** 0.5, 3)

    def get_successors(self, graph):
        '''
        Set successors for a node.
        Return successor nodes.
        Append the current node as a parent to successor nodes.
        '''
        i = 0
        if self.succ_set == False:
            for step in self.steps:
                node = graph[self.w + step[0]][self.h + step[1]]
                i += 1
                self.succ.append(node)
                # Add self node as the predecessor to the current successor node
                if node not in self.pred:
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
        # 1.0 distance in pixels to  close nodes
        if node.prob == 100:
            return np.inf
        elif node.prob == -1:
            return np.inf
        else:
            return ((self.w-node.w)**2+(self.h-node.h)**2)**0.5