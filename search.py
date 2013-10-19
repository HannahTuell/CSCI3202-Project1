# Project 1: Search (search.py, to accompany searchAgents.py)
# CSCI 3202 - Fall 2013
#
# Author: Hannah Tuell
# Collaborators: Justin Woodward


# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 

# educational purposes provided that (1) you do not distribute or publish 

# solutions, (2) you retain this notice, and (3) you provide clear 

# attribution to UC Berkeley, including a link to 

# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

# 

# Attribution Information: The Pacman AI projects were developed at UC Berkeley.

# The core projects and autograders were primarily created by John DeNero 

# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).

# Student side autograding was added by Brad Miller, Nick Hay, and 

# Pieter Abbeel (pabbeel@cs.berkeley.edu).



"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]


def depthFirstSearch(problem):
    from util import Stack
    closedSet = set() # set of nodes already visited
    fringe    = Stack() # stack that contains the fringe of the search
    path      = [] # list holding the path that will be returned for PacMan to follow
    
    # push the start state to the fringe
    state = problem.getStartState() 
    node  = makeNode(state) 
    fringe.push(node) 
    
    # step through the fringe until it is empty
    while not fringe.isEmpty():
        # update the information about the current node
        node  = fringe.pop()
        state = node.state
        path  = node.path
        
        # return the path if you reach the goal
        if problem.isGoalState(state):
            return path
            
        # if we have never visited this node before, expand it 
        # and add it's succesors to the fringe
        elif state not in closedSet:
            closedSet.add(state)
            successors = problem.getSuccessors(state)
            # start at the first successor and if it can not be followed to the end
            # continue to the next successor of the level before retreating back a step
            for each in successors:
                # get information from the successor
                nextState, nextAction, cost = each
                # update the path to include the successor
                nextPath  = list(path)
                nextPath.append(nextAction)
                # build a new node with the new successor's information and add it to the fringe
                nextNode = makeNode(nextState, nextPath)
                fringe.push(nextNode)
    
    # if we reach this point - there was no solution
    print "Depth First Search was unable to find a solution."
    return None


def breadthFirstSearch(problem):
    from util import Queue
    closedSet = set() # set of nodes already visited
    fringe    = Queue() # queue that contains the fringe of the search
    path      = [] # list holding the path that will be returned for PacMan to follow
    
    # push the start state to the fringe
    state = problem.getStartState() 
    node  = makeNode(state) 
    fringe.push(node) 
    
    # step through the fringe until it is empty
    while not fringe.isEmpty():
        # update the information about the current node and add it to the set
        node  = fringe.pop()
        state = node.state
        path  = node.path
        closedSet.add(state)

        # return the path if you reach the goal
        if problem.isGoalState(state):
            return path
            
        # add the node's successors to the fringe, start with the first successor
        # and expand all succesors before pursuing further nodes
        successors = problem.getSuccessors(state)
        for each in successors:
            # get information from the successor
            nextState, nextAction, cost = each
            if nextState not in closedSet:    
                closedSet.add(nextState)
                # update the path to include the successor
                nextPath  = []
                for every in path:
                    nextPath.append(every)
                nextPath.append(nextAction)
                # build a new node with the new successor's information and add it to the fringe
                nextNode = makeNode(nextState, nextPath)
                fringe.push(nextNode)
 
    # if we reach this point - there was no solution
    print "Breadth First Search was unable to find a solution."
    return None
   
def uniformCostSearch(problem):
    from util import PriorityQueue
    closedSet = set() # set of nodes already visited
    fringe    = PriorityQueue() # queue that contains the fringe of the search with its priorities
    path      = [] # list holding the path that will be returned for PacMan to follow
    cost      = 0 # will hold the cost
    
    # push the start state to the fringe
    state = problem.getStartState() 
    node  = makeNode(state) 
    fringe.push(node, cost)

    # step through the fringe until it is empty
    while not fringe.isEmpty():
        # update the information about the current node and add it to the set
        node  = fringe.pop()
        state = node.state
        path  = node.path
        cost  = node.cost
        
        # return the path if you reach the goal
        if problem.isGoalState(state):
            return path
            
        if state not in closedSet:
            closedSet.add(state)
            successors = problem.getSuccessors(state)
            # hold the cost just before the successors
            prevCost   = cost 
            for each in successors:
                # get information from the successor
                nextState, nextAction, nextCost = each
                cost = prevCost
                # update the path to include the successor
                nextPath = list(path)
                nextPath.append(nextAction)
                cost += nextCost
                # build a new node with the new successor's information and add it to the fringe
                nextNode = makeNode(nextState, nextPath, cost)
                fringe.push(nextNode, cost)
 
    # if we reach this point - there was no solution
    print "Uniform-Cost Search was unable to find a solution."
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    from util import PriorityQueue
    from util import PriorityQueueWithFunction
    
    closedSet = set() # set of nodes already visited
    fringe    = PriorityQueue() # queue that contains the fringe of the search with its priorities
    path      = [] # list holding the path that will be returned for PacMan to follow
    cost      = 0
  
    # calculate the priority level of the start state and push it to the fringe
    state = problem.getStartState() 
    h = heuristic(state, problem)
    priority = h + cost
    node = makeNode(state, path, cost, h)
    fringe.push(node, priority)
	
	# step through the fringe until it is empty
    while not fringe.isEmpty():
        # update the information about the current node and add it to the set
        node  = fringe.pop()
        state = node.state
        path  = node.path
        cost  = node.cost
        h     = node.h
        
        # return the path if you reach the goal
        if problem.isGoalState(state):
            return path
            
        if state not in closedSet:
            closedSet.add(state)
            successors = problem.getSuccessors(state)
            # hold the cost just before the successors
            prevCost   = cost 
            for each in successors:
                # get information from the successor
                nextState, nextAction, nextCost = each
                cost = prevCost
                # update the path to include the successor
                nextPath = list(path)
                nextPath.append(nextAction)
                cost += nextCost
                nextH = heuristic(nextState, problem)
                priority = nextH + cost
                # build a new node with the new successor's information and add it to the fringe
                nextNode = makeNode(nextState, nextPath, cost, h)
                fringe.push(nextNode, priority)

    # if we reach this point - there was no solution
    print "A* Search was unable to find a solution."
    return None
		
class makeNode(object):
    def __init__(self,state=None,path=[],cost=0, h=0):
        self.state = state
        self.path  = path
        self.cost  = cost
        self.h     = h
    def __eq__(self, item):
        if item == None: return False
        return (self.state == item.state)&(self.path==item.path)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch