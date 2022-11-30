# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
        state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    print "Start:", problem.getStartState()
    """
    "*** YOUR CODE HERE ***"
    frontier = util.Stack()
    node = Node(None, None, 0, problem.getStartState())
    frontier.push(node)
    exploredSet = []
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.statePointer):
            return node.reconstructPath()
        if node.statePointer not in exploredSet:
            exploredSet.append(node.statePointer)
            for successor in problem.getSuccessors(node.statePointer):
                childNode = Node(node, successor[1], successor[2], successor[0])
                frontier.push(childNode)
    raise RuntimeError("Iterative search failed to find a solution")

    
def dfsPriorityFunction(node):
    """ 
    The priorityQueue Function for DFS.
    Works by reversing the order of the passed node
    with respect to the other nodes, essentially
    last in first out.
    """
    return node.count * -1

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    frontier = util.Queue()
    node = Node(None, None, 0, problem.getStartState())
    frontier.push(node)
    exploredSet = []
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.statePointer):
            return node.reconstructPath()
        if node.statePointer not in exploredSet:
            exploredSet.append(node.statePointer)
            for successor in problem.getSuccessors(node.statePointer):
                childNode = Node(node, successor[1], successor[2], successor[0])
                frontier.push(childNode)
    raise RuntimeError("Iterative search failed to find a solution")

def bfsPriorityFunction(node):
    """ 
    The priorityQueue Function for BFS.
    Works by returning the order of the passed node
    with respect to the other nodes, essentially
    first in first out.
    """
    return node.count

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    node = Node(None, None, 0, problem.getStartState())
    frontier.push(node, node.pathCost)
    exploredSet = []
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.statePointer):
            return node.reconstructPath()
        if node.statePointer not in exploredSet:
            exploredSet.append(node.statePointer)
            for successor in problem.getSuccessors(node.statePointer):
                childNode = Node(node, successor[1], successor[2] + node.pathCost, successor[0])
                frontier.push(childNode, childNode.pathCost)
    raise RuntimeError("Iterative search failed to find a solution")

def ucsPriorityFunction(node):
    """ 
    The priorityQueue Function for UCS.
    Works by returning the pathCost from the passed node
    """
    return node.pathCost


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    return genericSearch(problem, algorithm = heuristic)

def genericSearch(problem, algorithm):
    frontier = util.PriorityQueueWithFunction()
    node = Node(None, None, 0, problem.getStartState())
    pathCost = 
    frontier.push(node)
    exploredSet = []
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.statePointer):
            return node.reconstructPath()
        if node.statePointer not in exploredSet:
            exploredSet.append(node.statePointer)
            for successor in problem.getSuccessors(node.statePointer):
                childNode = Node(node, successor[1], successor[2] + node.pathCost, successor[0])
                frontier.push(childNode)
    raise RuntimeError("Iterative search failed to find a solution")



def manhattanHeuristic(node):
    """ 
    The aStarSearch PriorityQueue Function.
    Works by adding the pathCost of the node
    to the manhattan distance from the current
    state to the goal. Assumes the goal is fixed
    at (1,1).
    """
    return util.manhattanDistance(node.statePointer, (1,1))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch




class Node:
    """ 
    Node class that's essentially a path to a certain state
    Keeps track of a parent, last action taken, the path-cost, 
    and a pointer to the state it ended at.
    """
    def __init__(self, parent, action, pathCost, state):
        self.parent = parent
        self.action = action
        self.pathCost = pathCost
        self.statePointer = state
    
    def __hash__(self):
        return hash(self.statePointer)

    def reconstructPath(self):
        """ 
        Reconstructs the path from the latest node to the first node when called.
        Also counts the path cost
        """
        if self.action == None: 
            return []
        actionList = []
        node = self
        while node.action != None: # Add all actions to a stack
            actionList.insert(0,node.action)
            node = node.parent
        return actionList
