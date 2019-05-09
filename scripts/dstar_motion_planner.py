#!/usr/bin/env python
import math
import sys
import ast
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np
import rospy
import random
import time
from copy import deepcopy
import heapq as hpq

import visualization_marker_const as vm
import linked_list_back as llb
import action_transition as at

class DSTARMotionPlanner:
    def __init__(self, in_state, gl_state, neigh_dist):
        self.init_state = in_state
        self.goal_state = gl_state
        self.pub_markers = rospy.Publisher('/markers', MarkerArray, queue_size=10)
        self.neigh_trans = []
        at.ActionTransition().neighbors_trans_precalc(len(self.init_state), neigh_dist, self.neigh_trans)
        self.maporigin = [0, 0]
        self.mapresol = 0
        self.occmap = OccupancyGrid()
        self.marker_array = []
	self.listofstates = []
        self.path_execution_complete = 'False'
        self.openset = []
        self.high_h = 10000

    def dist(self, x1, x2):
        return abs(x1[0]-x2[0]) + abs(x1[1]-x2[1])

    def dist_l1(self, x1, x2):
        return abs(x1[0]-x2[0]) + abs(x1[1]-x2[1])

    def dist_l2(self, x1, x2):
        return math.sqrt(pow(x1[0]-x2[0], 2) + pow(x1[1]-x2[1], 2))

    def insert(self, x, hnew):
        if hnew > self.high_h:
            hnew = self.high_h
        kx = 0    
        if x.t == 'NEW':
            kx = hnew
            hpq.heappush(self.openset, (kx, x))
        elif x.t == 'OPEN':

            #xtemp = self.openset.index((x.h,x)) #xtemp should be an index to the self.openset list
            for i in range(0, len(self.openset)):
                if self.openset[i][1] == x:
                    xtemp = i
                    break
            kx = min(self.openset[xtemp][0], hnew)

            #Remove x from the self.openset
            self.openset.pop(xtemp)
            #reorder the self.openset as a heap queue
            hpq.heapify(self.openset)

            #add the x element with the new key kx
            hpq.heappush(self.openset, (kx, x))

        elif x.t == 'CLOSED':

            kx = min(x.h, hnew)
            hpq.heappush(self.openset, (kx, x))
        x.h = hnew
        x.t = 'OPEN'

    #New goal given from rviz clicked point
    def newgoal(self, clickedgoal):
        print('New goal received at: ', [int((clickedgoal.point.y - self.maporigin[1])/self.mapresol), int((clickedgoal.point.x - self.maporigin[0])/self.mapresol)])
        if self.path_execution_complete == 'Pending':#If the robot received a new goal while it was executing a path, it should discard the new goal and continue its execution
            print 'Discarding new goal because path execution is still ongoing.'
            return
        else:
            #Reset goal state
            print 'New goal accepted'
            self.path_execution_complete = 'False'
            self.goal_state = [int((clickedgoal.point.y - self.maporigin[1])/self.mapresol), int((clickedgoal.point.x - self.maporigin[0])/self.mapresol)]

            #Reexecute planning callback
            self.callback(self.occmap)

    #Retrieve list of pointers of the path from the father of the goal until the initial state
    def get_backpointer_list(self, xcurrent, goalstate):
        print 'Getting backpointer list'
        p = []
        backtrackstate = goalstate.parent

        while backtrackstate: # TODO: Strangely, the xcurrent is not used. Perhaps it should enter in the loop termination condition
            #print 'backtrackstate ',backtrackstate.data, ', h = ',backtrackstate.h, ' occ = ',backtrackstate.occ

            path_marker = vm.vis_market_set(1, rospy.Time.now(), self.occmap.header.frame_id, self.marker_array.markers[-1].id + 1, Marker().SPHERE, Marker().ADD, Point(backtrackstate.data[1] * self.occmap.info.resolution + self.occmap.info.origin.position.x, backtrackstate.data[0] * self.occmap.info.resolution + self.occmap.info.origin.position.y, 0), Quaternion(0,0,0,1), Vector3(self.occmap.info.resolution*1.2, self.occmap.info.resolution*1.2, self.occmap.info.resolution*1.2), rospy.Duration(3), ColorRGBA(1,0,1,1))

            self.marker_array.markers.append(deepcopy(path_marker))
            p.append(backtrackstate)
            backtrackstate = backtrackstate.parent

        print 'Finished get_backpointer_list cycle'
        self.pub_markers.publish(self.marker_array)
        return p

    def process_state(self):
        if self.openset != []: #return x state with minimum k value
            x = hpq.heappop(self.openset)
        else:
            return -1
        kold = x[0]

        x[1].t = 'CLOSED'
        assert(self.listofstates[x[1].data[0], x[1].data[1]].t == x[1].t)

        hx = self.listofstates[x[1].data[0], x[1].data[1]].h 
        
        assert(x[1].h == hx)

        # Obtain neighboring states set
        neigbh_states_set = []
        #TODO: according to slide 48 of AppH-astar-dstar-Howie.pdf, we have to check all neighboring states, including occupied ones
        neigbh_states_set = at.ActionTransition(1).action_set(x[1].data, self.neigh_trans, self.state_space)

        #if len(neigbh_states_set) != 4:
        #    print 'Missing state!'

        #RAISE node:
        if kold < x[1].h  : 
            #print 'This should never happen if the robot executes the path without any whatsoever change to the original map'
            #print 'RAISE node'
            while neigbh_states_set != []:
                y_ind = 0
                y = neigbh_states_set[y_ind]

                ytemp = self.listofstates[y[0], y[1]]
                xtemp = x[1]
                hy = ytemp.h

                c_xy = self.dist_l2(xtemp.data, ytemp.data)
                if ytemp.occ == 100 or xtemp.occ == 100:
                    c_xy = self.high_h

                if ytemp.t != 'NEW' and hy <= kold and x[1].h  > hy + c_xy: #the 1 constant represents the edge cost between y and x
                    xtemp.parent = ytemp
                    xtemp.h = ytemp.h + c_xy
                    if xtemp.h > self.high_h:
                        xtemp.h = self.high_h
                neigbh_states_set.pop(y_ind)

	    #It is necessary to recalculate the neighboring states, so that they can be rechecked by by the code in LABEL2
            neigbh_states_set = at.ActionTransition(1).action_set(x[1].data, self.neigh_trans, self.state_space)

        #LOWER node:
        if kold == x[1].h :
            #print 'LOWER node'
            while neigbh_states_set != []:
                y_ind = 0
                y = neigbh_states_set[y_ind]

                ytemp = self.listofstates[y[0], y[1]]
                xtemp = x[1]
                hy = ytemp.h

                c_xy = self.dist_l2(xtemp.data, ytemp.data)
                if ytemp.occ == 100 or xtemp.occ == 100:
                    c_xy = self.high_h

                if ytemp.t == 'NEW' or (ytemp.parent == xtemp and hy != x[1].h  + c_xy) or (ytemp.parent != xtemp and hy > x[1].h  + c_xy):
                    ytemp.parent = xtemp
                    self.insert(ytemp, x[1].h  + c_xy)

                neigbh_states_set.pop(y_ind)   
        else: #LABEL2
            while neigbh_states_set != []:
                y_ind = 0
                y = neigbh_states_set[y_ind]

                ytemp = self.listofstates[y[0], y[1]]
                xtemp = x[1]
                hy = ytemp.h

                c_xy = self.dist_l2(xtemp.data, ytemp.data)
                if ytemp.occ == 100 or xtemp.occ == 100:
                    c_xy = self.high_h

                if ytemp.t == 'NEW' or (ytemp.parent == xtemp and hy != x[1].h  + c_xy):
                    ytemp.parent = xtemp
                    self.insert(ytemp, x[1].h  + c_xy)
                else:
                    if ytemp.parent != xtemp and hy > x[1].h  + c_xy:
                        self.insert(xtemp, x[1].h )
                    else:
                        if ytemp.parent != xtemp and x[1].h > hy + c_xy and ytemp.t == 'CLOSED' and hy > kold:
                            self.insert(ytemp, hy)

                neigbh_states_set.pop(y_ind)

	cur_state = x[1].data
	#print('Goal state not found. Next iteration.')

        bx = x[1].parent
        arrowdiff = []
        arrow_ori = Quaternion(0,0,0,1)

        #if bx != None:
        #    arrowdiff = [bx.data[i] - x[1].data[i] for i in [0,1]]

            #The arrowdiff is determined by considering the ordering of the cells in the map (x is the vertical axis and y is the horizontal axis)
            #In contrast, the correspodning quaternion is determined in the /map coordinate frame
            #^
       #     if arrowdiff == [1,0]:
       #         arrow_ori2 = quaternion_from_euler(0, 0, 3.14159/2, 'sxyz')
            #->
       #     elif arrowdiff == [0,1]:
       #         arrow_ori2 = quaternion_from_euler(0, 0, 0, 'sxyz')

	    #v
       #     elif arrowdiff == [-1,0]:
       #         arrow_ori2 = quaternion_from_euler(0, 0, -3.14159/2, 'sxyz')

	    #<-
       #     elif arrowdiff == [0,-1]:
       #         arrow_ori2 = quaternion_from_euler(0, 0, 3.14159, 'sxyz')
       #     arrow_ori.x = arrow_ori2[0]
       #     arrow_ori.y = arrow_ori2[1]
       #     arrow_ori.z = arrow_ori2[2]
       #     arrow_ori.w = arrow_ori2[3]            
        #print '2',arrow_ori
	##############################
	# White, state (history) marker
      
	cur_state_marker = vm.vis_market_set(1, rospy.Time.now(), self.occmap.header.frame_id, self.marker_array.markers[-1].id + 1, Marker().SPHERE, Marker().ADD, Point(cur_state[1] * self.occmap.info.resolution + self.occmap.info.origin.position.x, cur_state[0] * self.occmap.info.resolution + self.occmap.info.origin.position.y, 0), arrow_ori, Vector3(self.occmap.info.resolution,self.occmap.info.resolution,self.occmap.info.resolution), rospy.Duration(), ColorRGBA(1,1,1,1))
	#cur_state_marker.text = str(x[1].h)

	#If cur state marker was already visualized and now revisited, then remove the previous marker before adding the new
        if self.marker_array != []:
	    for i in range(1, len(self.marker_array.markers)-1):
		if self.marker_array.markers[i].pose.position.x == cur_state_marker.pose.position.x and self.marker_array.markers[i].pose.position.y == cur_state_marker.pose.position.y:
	            self.marker_array.markers[i].action = Marker().DELETE
		    break

	self.marker_array.markers.append(deepcopy(cur_state_marker))


	# Blue, current state marker
	cur_state_marker_blue = vm.vis_market_set(1, rospy.Time.now(), self.occmap.header.frame_id, self.marker_array.markers[-1].id + 1, Marker().SPHERE, Marker().ADD, Point(cur_state[1] * self.occmap.info.resolution + self.occmap.info.origin.position.x, cur_state[0] * self.occmap.info.resolution + self.occmap.info.origin.position.y, 0), Quaternion(0,0,0,1), Vector3(self.occmap.info.resolution,self.occmap.info.resolution,self.occmap.info.resolution), rospy.Duration(), ColorRGBA(0,0,1,1))

	self.marker_array.markers.append(deepcopy(cur_state_marker_blue))
	##############################

	self.pub_markers.publish(self.marker_array)
	self.marker_array.markers.pop()
        
        #print 'Published markers'

        if self.openset != []:
            return self.openset[0][0] #should return the kmin, under the condition that the self.openset is ordered
        else:
            return -1

    def callback(self, message):
	#Remove all previous markers from rviz, except the first two which correspond to the initial and goal state and the last which should correspond the last robot state
        if self.marker_array != []:
	    for i in range(2, len(self.marker_array.markers)-1):
	        self.marker_array.markers[i].action = Marker().DELETE
	    self.pub_markers.publish(self.marker_array)

        print('Received map')
        occmap = message
        newmap = message
        self.occmap = newmap

        self.maporigin = [newmap.info.origin.position.x, newmap.info.origin.position.y]
        self.mapresol = newmap.info.resolution

        #If this is a new attempt to search for the goal, initialize all states using default constructor (all states are set to 'NEW'). Otherwise, if it just an update of the map during the execution of the path, we do not recreate new states, but we only reset the occupancy values
        if self.listofstates == [] or self.path_execution_complete == 'False':
            self.listofstates = np.asarray([[llb.MyListNode() for k in range(0, occmap.info.width)] for l in range(0, occmap.info.height)])
	#print self.listofstates.shape
        print('Updating map...')
        self.state_space = np.ones((occmap.info.height, occmap.info.width))
        for i in range(0, occmap.info.height):
	    for j in range(0, occmap.info.width):

                #This is part of the modify_cost function
		#If the map has changed
		if (self.listofstates[i,j].occ != occmap.data[i*occmap.info.width + j]):
                    if self.listofstates[i,j].t == 'CLOSED':
                        if occmap.data[i*occmap.info.width + j] == 0: #If the new state is now free
                            self.insert(self.listofstates[i,j], self.listofstates[i,j].h)
                            print 'This should happen ONLY if cells have been deoccupied'
                        elif occmap.data[i*occmap.info.width + j] == 100: #If the new state is now occupied
                            self.insert(self.listofstates[i,j], self.high_h)
                    
	            neigbh_states_set = []

                    #Take all possible neighbors of current state, including occupied ones
	            neigbh_states_set = at.ActionTransition(1).action_set([i,j], self.neigh_trans, self.state_space)

	            while neigbh_states_set != []:
	                y_ind = 0
	                y = neigbh_states_set[y_ind]

	                ytemp = self.listofstates[y[0], y[1]]
                        if ytemp.t == 'CLOSED':
                            if occmap.data[y[0]*occmap.info.width + y[1]] == 0: #If it is currently a free state, then just put it back to the open by keeping the same h value
                                self.insert(ytemp, ytemp.h)
                                #print 'This should happen ONLY if neighboring cells are not occupied'
                            else: #if it is currently an occupied state, then put it back to the open but with a very high h value
                                self.insert(ytemp, self.high_h)
	                neigbh_states_set.pop(y_ind) #take new state

	        if occmap.data[i*occmap.info.width + j] == 0:
	            self.state_space[i,j] = 0
                    self.listofstates[i,j].occ = 0
	        else:
	            self.state_space[i,j] = 100
                    self.listofstates[i,j].occ = 100
                self.listofstates[i,j].data = [i,j]

                #print self.listofstates[i,j].data
                #sys.stdin.read(1)
	print('Map was updated')
        self.marker_array = MarkerArray()
 
        ################################
        # Visualization Markers creation
        # Initial state marker
        init_state_marker = vm.vis_market_set(0, rospy.Time.now(), newmap.header.frame_id, 0, Marker().SPHERE, Marker().ADD, Point(self.init_state[1] * newmap.info.resolution + newmap.info.origin.position.x, self.init_state[0] * newmap.info.resolution + newmap.info.origin.position.y, 0), Quaternion(0,0,0,1), Vector3(self.occmap.info.resolution*2,self.occmap.info.resolution*2,self.occmap.info.resolution*2), rospy.Duration(), ColorRGBA(0,1,0,1))
        self.marker_array.markers.append(init_state_marker)

        # Goal state marker
        goal_state_marker = vm.vis_market_set(1, rospy.Time.now(), newmap.header.frame_id, self.marker_array.markers[-1].id + 1, Marker().SPHERE, Marker().ADD, Point(self.goal_state[1] * newmap.info.resolution + newmap.info.origin.position.x, self.goal_state[0] * newmap.info.resolution + newmap.info.origin.position.y, 0), Quaternion(0,0,0,1), Vector3(self.occmap.info.resolution*2,self.occmap.info.resolution*2,self.occmap.info.resolution*2), rospy.Duration(), ColorRGBA(1,0,0,1))

        self.marker_array.markers.append(goal_state_marker)
        self.pub_markers.publish(self.marker_array)
        ################################


        if self.path_execution_complete == 'False':
            self.dstar_main()
        elif self.path_execution_complete == 'Pending':
            print 'Map has been updated with the new information'
            return

    def dstar_main(self):
        self.path_execution_complete = 'Pending'
	self.openset = []

	g = self.listofstates[self.goal_state[0], self.goal_state[1]]

	self.insert(g, 0)	
	x = self.listofstates[self.init_state[0], self.init_state[1]]

	p = self.init_plan(x, g) 
        print 'Initial plan has returned'

	if p == None:
	    return None
	
        #From this point onwards, the robot is supposed to start executing the plan. In other words, it starts from the initial position x and follows the backpointers until it reaches the goal. Along the way, the map may change. After each state advance, the robot updates (if necessary) the map and repairs the path.

        print 'There are still ',len(self.openset),' states that are open'
        print 'Executing path'
	while x != g and x != None:
            print 'Current state is: ', x.data

	    # Orange, current state marker
	    cur_state_marker_orange = vm.vis_market_set(1, rospy.Time.now(), self.occmap.header.frame_id, self.marker_array.markers[-1].id + 1, Marker().SPHERE, Marker().ADD, Point(x.data[1] * self.occmap.info.resolution + self.occmap.info.origin.position.x, x.data[0] * self.occmap.info.resolution + self.occmap.info.origin.position.y, 0), Quaternion(0,0,0,1), Vector3(self.occmap.info.resolution*1.2,self.occmap.info.resolution*1.2,self.occmap.info.resolution*1.2), rospy.Duration(), ColorRGBA(0.2,0.2,0.2,1))
	    self.marker_array.markers.append(deepcopy(cur_state_marker_orange))
	    self.pub_markers.publish(self.marker_array)

            sys.stdin.read(1)
            #prepare_repair(self.openset, x)

	    p = self.repair_replan(x, g)
	    if p == None:
                print 'No plan to reach to goal. Waiting for new map or goal to reach'
	        return None

	    #x = p[0].parent
            x = p[0] #Move to the next state of the path
            if p[0].occ == 100:
                print 'A collision is about to happen if the robots transits to the next state'
        print 'Path has been executed :-)'
        self.path_execution_complete = 'True'
	return x
        ######################

    def init_plan(self, xcurrent, goalstate):
        print 'Evaluating init_plan'
        kmin = 0
	num_of_transitions = 0
        while kmin != -1 and xcurrent.t != 'CLOSED':
            num_of_transitions += 1
	    if num_of_transitions == 5:
		print num_of_transitions
		sys.stdin.read(1)
	    elif num_of_transitions == 50:
		print num_of_transitions
		sys.stdin.read(1)
	    elif num_of_transitions == 250:
		print num_of_transitions
		sys.stdin.read(1)
	    elif num_of_transitions == 1000:
		print num_of_transitions
		sys.stdin.read(1)
            kmin = self.process_state()
        p = self.get_backpointer_list(goalstate, xcurrent)
        print 'Finished init_plan'
        print num_of_transitions
        return p

    def repair_replan(self, x, g):
        #hx = x.h
        kmin = self.openset[0][0]
        print 'hx=',x.h,' of state ', x.data

        if kmin < x.h and kmin != -1:
            print 'A new path has to be calculated, kmin = ',kmin
        while kmin < x.h and kmin != -1: #If the estimated cost to goal from the current state is higher than the smallest possible cost from the list of open states, then we have to expand
            kmin = self.process_state()
        print 'kmin=',kmin,' Replanning completed. Republishing new path'
        p = self.get_backpointer_list(g, x)
        return p
                
if __name__ == '__main__':
    try:
        # Retrieve the initial and goal states as n-dimensional vectors enclosed in parentheses
        istate = list(ast.literal_eval(sys.argv[1]))
        gstate = list(ast.literal_eval(sys.argv[2]))
        neigh_connectivity = int(sys.argv[3])

        # Create Breadth First Search motion planner object with the given initial and global state
        myDSTAR = DSTARMotionPlanner(istate, gstate, neigh_connectivity)

        rospy.init_node('DSTAR_motion_planner', anonymous=True)
        rospy.Subscriber('/map', OccupancyGrid, myDSTAR.callback)
        rospy.Subscriber('/clicked_point', PointStamped, myDSTAR.newgoal)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
