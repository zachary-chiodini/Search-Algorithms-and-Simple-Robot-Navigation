from graph import Graph
from problem import Problem, Radius, State, Substate
from tree import Node

from typing import List, Tuple, Union


def greedy_search(
        init: State, goal: State, r: Radius, m: int, n: int
        ) -> Union[str, List[State]]:

    def magnitude(v: Tuple[int, int]) -> float:
        """Magnitude of vector v."""
        return (v[0]**2 + v[1]**2)**0.5

    def difference(sub_f: Substate, sub_i: Substate) -> Tuple[int, int]:
        """Vector difference between substate f and i."""
        return sub_f[0] - sub_i[0], sub_f[1] - sub_i[1]

    def distance(sub_i: Substate, sub_f: Substate) -> float:
        """Straight line distance from substate i to f."""
        return magnitude(difference(sub_f, sub_i))

    def heuristic(state_n: State, goal_: State):
        """
        Sum of estimated distance each robot j has to
        travel from substate j to the goal substate j.
        """
        return sum(distance(state_n[j], goal_[j])
                   for j in range(problem.robots))

    graph = Graph()
    problem = Problem(init, goal, r, m, n)
    node_id = 0  # node identifier for graph
    sibling = 0  # sibling identifier for graph
    frontier = {heuristic(init, goal): [Node(init)]}
    explored = set()
    while True:
        if not frontier:
            return 'Failure'
        # Priority queue
        h_min = min(frontier.keys())
        node = frontier[h_min].pop()
        if not frontier[h_min]:
            del frontier[h_min]
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
                h = heuristic(child.state, goal)
                if h in frontier.keys():
                    frontier[h].append(child)
                else:
                    frontier[h] = [child]
                sibling += 1
        sibling = 0
        graph.tree(node, children)
        graph.grid(node, m, n, children)
