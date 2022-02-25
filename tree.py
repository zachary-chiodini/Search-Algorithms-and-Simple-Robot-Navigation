from typing import List, Optional, TypedDict

from problem import State


class Path(TypedDict):
    number: List[int]
    path: List[State]


class Node:

    def __init__(self, state: State, parent: Optional['Node'] = None,
                 children=0, pathcost=0.0, sibling=0, number=0):
        """Create a search tree node"""
        self.state = state
        self.parent = parent
        self.children = children
        self.pathcost = pathcost
        self.sibling = sibling
        self.number = number

    @staticmethod
    def lineage(child: 'Node') -> int:
        """
        Generates the product of the count of all
        parent nodes and their children, used to
        constrict the spread of the lower branches
        of the tree when graphing.
        """
        n = 1
        while child.parent:
            parent = child.parent
            n *= parent.children
            child = parent
        return n

    @staticmethod
    def path_to(child: 'Node') -> Path:
        """
        Returns a dictionary of the chosen states
        and identifiers for the chosen nodes.
        """
        path: Path = {'number': [child.number], 'path': [child.state]}
        while child.parent:
            parent = child.parent
            path['number'].insert(0, parent.number)
            path['path'].insert(0, parent.state)
            child = parent
        return path
