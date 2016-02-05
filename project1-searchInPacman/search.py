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

from game import Directions
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
    """
    frontier = util.Stack()

    start = problem.getStartState()
    frontier.push((start, None, 1))
    visited = set([start])

    camefrom = {} # Camefrom [successor, camefrom]

    while not frontier.isEmpty():
        current = frontier.pop()

        if problem.isGoalState(current[0]):
            goal = current
            break

        for successor in problem.getSuccessors(current[0]):
            if successor[0] not in visited:
                frontier.push(successor)
                visited.add(current[0])
                camefrom.update({successor : current})

    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState(), goal, camefrom)



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()

    start = problem.getStartState()
    frontier.push((start, None, 1))
    visited = set([start])

    camefrom = {} # Camefrom (successor, camefrom)

    while not frontier.isEmpty():
        current = frontier.pop()
        currentPos = current[0]

        if problem.isGoalState(currentPos):
            goal = current
            break

        for successor in problem.getSuccessors(currentPos):
            successorPos = successor[0]
            if successorPos not in visited:
                frontier.push(successor)
                visited.add(successorPos)
                camefrom.update({successor : current})

    # Retrace steps by the states ancestors
    return buildPath(problem.getStartState(), goal, camefrom)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()
    startPos = problem.getStartState()
    startState = (startPos, None, 0)
    frontier.push(startState, 0)
    distance = {startPos: 0}

    visited = set([])
    camefrom = {}

    while not frontier.isEmpty():
      current = frontier.pop()
      currentPos = current[0]
      visited.add(currentPos)

      if problem.isGoalState(currentPos):
        goal = current
        break

      for neighbor in problem.getSuccessors(currentPos):
        neighborPos = neighbor[0]
        neighborTotalCost = distance[currentPos] + neighbor[2]
        if neighborPos in distance and distance[neighborPos] > neighborTotalCost:
          distance[neighborPos] = neighborTotalCost
          camefrom[neighbor] = current
        if neighborPos not in visited:
          distance[neighborPos] = neighborTotalCost
          camefrom[neighbor] = current
          frontier.push(neighbor, neighborTotalCost)

    ## Retrace steps by the states ancestors
    return buildPath(startPos, goal, camefrom)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    frontier = util.PriorityQueue()

    start = problem.getStartState()
    startState = (start, None, 0)
    frontier.push(startState, 0)

    visited = {start:True}
    distance = {startState:0}
    camefrom = {} # Camefrom (successor, camefrom)

    path = [] # path to retrace steps

    while not frontier.isEmpty():
        current = frontier.pop()
        #import pdb; pdb.set_trace()

        if(problem.isGoalState(current[0])):
            goal = current
            # Retrace steps by the states ancestors
            return buildPath(problem.getStartState(), goal, camefrom)


        for successor in problem.getSuccessors(current[0]):
            if successor[0] not in visited:
                g = distance[current] + successor[2]
                h = heuristic(successor[0], problem)

                visited[current[0]] = True
                tempCost = g + h
                if successor not in distance:
                    distance[successor] = g
                    frontier.push(successor, tempCost)
                    camefrom.update({successor : current})

    
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
