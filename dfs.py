from graph import Graph
from problem import Problem, Radius, State
from tree import Node

from typing import List, Union


def depth_first_search(
        init: State, goal: State, r: Radius, m: int, n: int
        ) -> Union[str, List[State]]:
    graph = Graph()
    problem = Problem(init, goal, r, m, n)
    node_id = 0  # node identifier for graph
    sibling = 0  # sibling identifier for graph
    frontier = [Node(init)]
    explored = set()
    while True:
        if not frontier:
            return 'Failure'
        node = frontier.pop(-1)  # LIFO queue
        graph.tree(node)
        graph.grid(node, m, n)
        if problem.goal_test(node.state):
            return node.path_to(node)['path']
        explored.add(node.state)
        children = []
        for state in problem.successor(node.state):
            child = Node(state, parent=node)
            if child.state not in explored:
                node_id += 1
                node.children += 1
                child.number = node_id
                child.sibling = sibling
                children.append(child)
                frontier.append(child)
                sibling += 1
        sibling = 0
        graph.tree(node, children)
        graph.grid(node, m, n, children)
