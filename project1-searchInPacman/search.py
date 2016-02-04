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
    from game import Directions
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from game import Directions    
    #return [Directions.NORTH]
    
    frontier = util.Stack()

    start = problem.getStartState()
    frontier.push((start, None, 1))

    visited = [start]
    camefrom = {} # Camefrom [successor, camefrom]

    path = [] # path that solves the problem

    while not frontier.isEmpty():
        current = frontier.pop()
        
        if(problem.isGoalState(current[0])):        
            goal = current
            break

        if current[0] not in visited:
                visited.append(current[0])

        for successor in problem.getSuccessors(current[0]):
            if successor[0] not in visited:
                frontier.push(successor) 
                camefrom.update({successor : current})

    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState(), goal, camefrom)



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from game import Directions    
    
    frontier = util.Queue()

    start = problem.getStartState()
    frontier.push((start, None, 1))

    visited = [start]
    camefrom = {} # Camefrom (successor, camefrom)

    path = [] # path that solves the problem

    while not frontier.isEmpty():
        current = frontier.pop()
        
        if(problem.isGoalState(current[0])):        
            goal = current
            break

        for successor in problem.getSuccessors(current[0]):
            if successor[0] not in visited:
                frontier.push(successor)
                visited.append(successor[0])
                camefrom.update({successor : current})

    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState(), goal, camefrom)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"        
    frontier = util.PriorityQueue()

    start = problem.getStartState()
    startState = (start, None, 1)
    frontier.push(startState, 0)

    visited = [start]
    distance = {startState:0}
    camefrom = {} # Camefrom (successor, camefrom)

    path = [] # path that solves the problem

    while not frontier.isEmpty():
        current = frontier.pop()
        #import pdb; pdb.set_trace()

        if(problem.isGoalState(current[0])):
            goal = current
            break

        #visited.append(current[0])

        for successor in problem.getSuccessors(current[0]):
            if successor[0] not in visited:
                tempCost = distance[current] + successor[2]
                
                if successor not in distance or tempCost < distance[successor]:
                    distance[successor] = tempCost
                    frontier.push(successor, tempCost)
                    visited.append(successor[0])
                    camefrom.update({successor : current})
        
    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState(), goal, camefrom)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
    """ ATEMPT ON A*
    frontier = util.PriorityQueue()

    start = problem.getStartState()
    startNode = (start, None, 1) # Create a starting node (looks like a successor

    frontier.push(startNode, 1) # Add start node to frontier

    visited = [start] # Mark start node as visited

    camefrom = {} # Camefrom (successor, camefrom)

    path = [] # path that solves the problem

    cost = {startNode:0}# Score to best path
    hCost = {start : heuristic(start, problem)}
    
    while not frontier.isEmpty():
        current = frontier.pop()
        
        if(problem.isGoalState(current[0])):        
            goal = current # Don't break maybe just record it
            break

        #visited.append(neighbor[0]) # Mark as visited
        visited.append(current[0])

        neighbors = problem.getSuccessors(current[0])
        for neighbor in neighbors:
            if neighbor[0] not in visited:

                # The combined cost of path and heuristic should be the priority for the queue
                neighborCost = neighbor[2]
                tempScore = cost[current] + neighborCost # score of the current's score + the distance to the neighbor

                if neighbor not in frontier:
                    frontier.push(neighbor, neighbor[2])
                elif tempScore < hCost[neighbor]:
                    camefrom.update({neighbor : current})
                    cost[neighbor] = tempScore
                    nState = neighbor[0]
                    hCost[neighbor] = score[neighbor] + heuristic(nState, problem)
                                        

    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState, goal, camefrom)
"""

"""
A* search

- Expand on the lowest cost node
- Once a node has been expanded add it to a closed node list
- Don't exit when a goal node is found, but rather add it to the queue and when it becomes the lowest cost and it gets dequed then that's the goal.

"""

    
def buildPath(start, goal, camefrom):
    path = []
    c = goal
    while c[0] != start:
        path.insert(0, c[1])
        c = camefrom[c]

    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
