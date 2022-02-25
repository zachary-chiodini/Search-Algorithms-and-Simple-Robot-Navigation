from graph import Graph
from problem import Problem, Radius, State, Substate
from tree import Node

from typing import List, Tuple, Union


Vector = Tuple[float, float]


def astar_search(
        init: State, goal: State, r: Radius, m: int, n: int
        ) -> Union[str, List[State]]:

    """Modified A* search"""

    def magnitude(v: Vector) -> float:
        """Magnitude of vector v."""
        return (v[0]**2 + v[1]**2)**0.5

    def difference(vf: Vector, vi: Vector) -> Vector:
        """Vector difference between vector vf and vi."""
        return vf[0] - vi[0], vf[1] - vi[1]

    def distance(sub_i: Substate, sub_f: Substate) -> float:
        """Straight line distance from substate i to f."""
        return magnitude(difference(sub_f, sub_i))

    def set_mag(c: float, v: Vector) -> Vector:
        """Change the magnitude of vector v to the value c."""
        mag_v = magnitude(v)
        if mag_v:
            nx, ny = v[0] / mag_v, v[1] / mag_v
            return c * nx, c * ny
        return v

    def h_score(state_n: State, goal_: State):
        """"
        Sum of estimated distance each robot j has to
        travel from substate j to the goal substate j.
        """
        return sum(distance(state_n[j], goal_[j])
                   for j in range(problem.robots))

    def g_score(child_: Node, goal_: State):
        """
        The g score is the path cost function, which is
        the sum of all step costs along the current path.
        The step cost for a child node is the sum of the
        straight line distance from each parent substate
        to their child substate plus the magnitude of the
        difference of two vectors that serve to increase
        the cost of steps that are idle or directed away
        from the goal.
        """
        stepcost = 0
        distance_ = 0
        for j in range(problem.robots):
            # position vector relative to goal substate (heuristic vector)
            h_vector = difference(child_.state[j], goal_[j])
            # vector pointing from child to parent substate (step cost vector)
            s_vector = difference(child_.parent.state[j], child_.state[j])
            distance_ += magnitude(s_vector)
            if magnitude(s_vector) == 0:
                s_vector = -h_vector[0], -h_vector[1]
            else:
                s_vector = set_mag(magnitude(h_vector), s_vector)
            stepcost += magnitude(difference(h_vector, s_vector))
        return child_.parent.pathcost + stepcost + distance_

    def f_score(child_: Node, goal_: State):
        """Total of g_score plus h_score."""
        return child_.pathcost + h_score(child_.state, goal_)

    graph = Graph()
    problem = Problem(init, goal, r, m, n)
    node_id = 0  # node identifier for graph
    sibling = 0  # sibling identifier for graph
    frontier = {h_score(init, goal): [Node(init)]}
    explored = set()
    while True:
        if not frontier:
            return 'Failure'
        # Priority queue
        f_min = min(frontier.keys())
        node = frontier[f_min].pop(0)
        if not frontier[f_min]:
            del frontier[f_min]
        graph.tree(node)
        graph.grid(node, m, n)
        if problem.goal_test(node.state):
            return node.path_to(node)['path']
        explored.add(node.state)
        children = []
        for state in problem.successor(node.state):
            child = Node(state, parent=node)
            child.pathcost = g_score(child, goal)
            if child.state not in explored:
                node_id += 1
                node.children += 1
                child.number = node_id
                child.sibling = sibling
                children.append(child)
                f = f_score(child, goal)
                if f in frontier.keys():
                    frontier[f].append(child)
                else:
                    frontier[f] = [child]
                sibling += 1
        sibling = 0
        graph.tree(node, children)
        graph.grid(node, m, n, children)
