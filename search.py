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
    from util import Stack

    stack = Stack()
    path = []
    visited_nodes = set()
    first_pos = (problem.getStartState(), path, 0)
    stack.push(first_pos)
    while not stack.isEmpty():
        cur_fringe = stack.pop()
        if cur_fringe[0] in visited_nodes:
            continue
        if problem.isGoalState(cur_fringe[0]):
            path = cur_fringe[1]
            break

        visited_nodes.add(cur_fringe[0])
        fringe_level = problem.getSuccessors(cur_fringe[0])
        for fringe in fringe_level:
            if fringe[0] not in visited_nodes:
                stack.push((fringe[0], cur_fringe[1] + [fringe[1]], fringe[2]))

    return path

    # visited_nodes = set()
    # key = ''.join(str(x) for x in problem.getStartState())
    # visited_nodes.add(key)
    # directions = []
    # res = ''
    # path_map = []
    # count = 0
    # stack = Stack()
    # fringe_level = problem.getSuccessors(problem.getStartState())
    # for fringe in fringe_level:
    #     stack.push(fringe)
    # while not stack.isEmpty():
    #     cur_pos = stack.pop()
    #     key = ''.join(str(x) for x in cur_pos[0])
    #     if key in visited_nodes:
    #         count += 1
    #         path_map[count] = []
    #         continue

    #     path_map[count].append(cur_pos[1])

    #     visited_nodes.add(key)
    #     directions.append(cur_pos[1])
    #     if problem.isGoalState(cur_pos[0]):
    #         res = path_map[count]
    #         break
    #     fringe_level = problem.getSuccessors(cur_pos[0])
    #     for fringe in reversed(fringe_level):
    #         stack.push(fringe)

    # print directions
    # print res
    # return directions
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    from util import Queue

    queue = Queue()
    path = []
    visited_nodes = set()
    first_pos = (problem.getStartState(), path, 0)
    queue.push(first_pos)
    while not queue.isEmpty():
        cur_fringe = queue.pop()
        if cur_fringe[0] in visited_nodes:
            continue
        if problem.isGoalState(cur_fringe[0]):
            path = cur_fringe[1]
            break

        visited_nodes.add(cur_fringe[0])
        fringe_level = problem.getSuccessors(cur_fringe[0])
        for fringe in fringe_level:
            if fringe[0] not in visited_nodes:
                queue.push((fringe[0], cur_fringe[1] + [fringe[1]], fringe[2]))

    return path

    """visited_nodes = set()
    key = ''.join(str(x) for x in problem.getStartState())
    visited_nodes.add(key)
    directions = []
    queue = Queue()
    fringe_level = problem.getSuccessors(problem.getStartState())
    for fringe in fringe_level:
        queue.push(fringe)
    while not queue.isEmpty():
        cur_pos = queue.pop()
        key = ''.join(str(x) for x in cur_pos[0])
        if key in visited_nodes:
            continue
        print key

        visited_nodes.add(key)
        if problem.isGoalState(cur_pos[0]):
            break
        directions.append(cur_pos[1])
        fringe_level = problem.getSuccessors(cur_pos[0])
        print fringe_level
        for fringe in fringe_level:
            queue.push(fringe)

    print directions
    return directions"""
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from util import PriorityQueue

    queue = PriorityQueue()
    path = []
    cost = 0
    visited_nodes = set()
    first_pos = (problem.getStartState(), path, 0)
    queue.push(first_pos, cost)
    while not queue.isEmpty():
        cur_fringe = queue.pop()
        if cur_fringe[0] in visited_nodes:
            continue
        if problem.isGoalState(cur_fringe[0]):
            path = cur_fringe[1]
            break

        visited_nodes.add(cur_fringe[0])
        fringe_level = problem.getSuccessors(cur_fringe[0])
        for fringe in fringe_level:
            if fringe[0] not in visited_nodes:
                cost = cur_fringe[2] + fringe[2]
                queue.push((fringe[0], cur_fringe[1] + [fringe[1]], fringe[2]), cost)

    return path
    # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue

    queue = PriorityQueue()
    path = []
    cost = 0
    visited_nodes = set()
    first_pos = (problem.getStartState(), path, 0)
    queue.push(first_pos, cost)
    while not queue.isEmpty():
        cur_fringe = queue.pop()
        if cur_fringe[0] in visited_nodes:
            continue
        if problem.isGoalState(cur_fringe[0]):
            path = cur_fringe[1]
            break

        visited_nodes.add(cur_fringe[0])
        fringe_level = problem.getSuccessors(cur_fringe[0])
        for fringe in fringe_level:
            if fringe[0] not in visited_nodes:
                cost = cur_fringe[2] + fringe[2]
                upcoming_cost = heuristic(fringe[0], problem)
                total_cost = cost + upcoming_cost
                queue.push((fringe[0], cur_fringe[1] + [fringe[1]], fringe[2]), total_cost)

    return path
    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
