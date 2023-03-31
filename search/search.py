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


import queue

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class MyNode:
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

    def __eq__(self, other):
        return self.state == other.state

    def __str__(self):
        return self.state


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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    frontier = util.Stack()
    expended = []
    frontier.push(MyNode(problem.getStartState(), None, None, 0))
    while True:
        if frontier.isEmpty():
            return
        node = frontier.pop()
        state = node.state
        print(f'expandeaza {node.state} cu parintele ->  {node.parent.__str__()}')
        expended.append(state)

        if problem.isGoalState(node.state):
            path = []
            pathS = []
            while node.state != problem.getStartState():
                pathS.append(node.state)
                path.append(node.action)
                node = node.parent
            path.reverse()
            pathS.reverse()
            print(f'path: {pathS} ')
            return path

       # print(f'successors for {node.state}: {problem.getSuccessors(node.state)}')
        for (successor, action, cost) in problem.getSuccessors(node.state):
            if successor not in expended:
                print(f'se va expanda {successor} cu action {action}')
                frontier.push(MyNode(successor, node, action, cost))
            else:
                print(f'nu se va expanda {successor}')
        print(f'frontiera: {frontier.__str__()}')

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "* YOUR CODE HERE *"

    frontier = util.Queue()
    expended = []
    frontier.push(MyNode(problem.getStartState(), None, None, 0))
    while True:
        if frontier.isEmpty():
            return
        node = frontier.pop()
        state = node.state
        print(f'expandeaza {node.state} cu actiunea -> {node.action}')
        expended.append(state)

        if problem.isGoalState(node.state):
            path = []
            pathS = []
            while node.state != problem.getStartState():
                pathS.append(node.state)
                path.append(node.action)
                node = node.parent
            path.reverse()
            pathS.reverse()
            print(f'path: {pathS} ')
            return path

        print(f'successors for {node.state}: {problem.getSuccessors(node.state)}')
        for (successor, action, cost) in problem.getSuccessors(node.state):
            if successor not in expended and MyNode(successor, node, action, cost) not in frontier.list:
                print(f'se va expanda {successor} cu action {action}')
                frontier.push(MyNode(successor, node, action, cost))
            else:
                print(f'nu se va expanda {successor}')
        print(f'frontiera: {frontier.__str__()}')

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"



    frontier = util.PriorityQueue()
    expended = []
    frontier.push(MyNode(problem.getStartState(), None, None, 0), 0)
    while True:
        if frontier.isEmpty():
            return
        node = frontier.pop()
        print(f'expandeaza {node.state} cu actiunea -> {node.action}')
        state = node.state

        expended.append(state)

        if problem.isGoalState(node.state):
            path = []
            pathS = []
            while node.state != problem.getStartState():
                pathS.append(node.state)
                path.append(node.action)
                node = node.parent
            path.reverse()
            pathS.reverse()
            print(f'path: {pathS} ')
            return path

        print(f'successors for {node.state}: {problem.getSuccessors(node.state)}')
        for (successor, action, cost) in problem.getSuccessors(node.state):
            if successor not in expended and successor not in frontier.heap:
                print(f'se va expanda {successor} cu action {action}')
                frontier.update(MyNode(successor, node, action, node.cost + cost), node.cost + cost)
            else:
                print(f'nu se va expanda {successor}')
        print(f'frontiera: {frontier.__str__()}')

        """
        frontier = lista de prioritati in functie de cost
        f(n)  = g(n) 
        
        
        """



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.
    la fel ca UNIFORM COST
    prioritate f(n) = g(n) + h(n)
    priority q data de costul de la start pana la n + costul estimat de la n pana la  goal
    (o estimare a caii complete)
    cu cat H(n) mai apraope de costul real e cu atat mai bine h(n) cat mai mare dar mai mic decat distanta reala
    h il scoatem din problema de cautare
    h cu ajutorul distrantei manhatten(mai buna)/ distanta euclidiana








    """
    "*** YOUR CODE HERE ***"
    # frontier = util.PriorityQueue()
    # expended = []
    # distances = {}
    # frontier.push(MyNode(problem.getStartState(), None, None, 0), heuristic(problem.getStartState(), problem))
    # distances[problem.getStartState()] = 0
    # while True:
    #     if frontier.isEmpty():
    #         return
    #     node = frontier.pop()
    #     state = node.state
    #     expended.append(state)
    #     if problem.isGoalState(node.state):
    #         path = []
    #         while node.state != problem.getStartState():
    #             path.append(node.action)
    #             node = node.parent
    #         path.reverse()
    #         return path
    #     for (successor, action, cost) in problem.getSuccessors(state):
    #         if successor in expended:
    #             continue
    #
    #         if node.cost + cost < distances.get(successor, float('inf')):
    #             distances[successor] = node.cost + cost + heuristic(successor, problem)
    #             frontier.update(MyNode(successor, node, action, node.cost + cost), node.cost + cost + heuristic(successor, problem))


    frontier = util.PriorityQueue()
    expended = []
    distances = []
    frontier.push(MyNode(problem.getStartState(), None, None, 0), heuristic(problem.getStartState(), problem))
    distances.append((problem.getStartState(), 0))
    while True:
        node = frontier.pop()
        state = node.state
        expended.append(state)
        if problem.isGoalState(node.state):
            path = []
            while node.state != problem.getStartState():
                path.append(node.action)
                node = node.parent
            path.reverse()
            return path
        for (successor, action, cost) in problem.getSuccessors(state):
            if successor not in expended and successor not in frontier.heap:


            # dist = [(i, tpl[1]) for i, tpl in enumerate(distances) if tpl[0] == successor]

            # if not len(dist):
            #     distances.append((successor, node.cost + cost + heuristic(successor, problem)))
                frontier.update(MyNode(successor, node, action, node.cost + cost), node.cost + cost + heuristic(successor, problem))
            # elif node.cost + cost < dist[0][1] and dist[0] in distances:
            #     # distances.remove(dist[0])
            #     # distances.insert(dist[0][0], node.cost + cost + heuristic(successor, problem))
            #     distances[dist[0][0]]=(successor,node.cost + cost + heuristic(successor, problem))
            #     frontier.update(MyNode(successor, node, action, node.cost + cost),
            #                     node.cost + cost + heuristic(successor, problem))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
