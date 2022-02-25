from typing import List, Optional, Tuple


class Node:

    Substate = Tuple[int, int]
    State = List[Substate]

    def __init__(self, state: State, parent: Optional['Node'] = None,
                 children=0, pathcost=0.0, sibling=0, number=0):
        """Create a search tree node."""
        self.state = state
        self.parent = parent
        self.children = children
        self.pathcost = pathcost
        self.sibling = sibling
        self.number = number

    @staticmethod
    def reverse(self, list_: List):
        return list_[::-1]

    @staticmethod
    def lineage(self, child: 'Node'):
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

    def path(self, child: 'Node'):
        """
        Returns a dictionary of the chosen states
        and identifiers for the chosen nodes.
        """
        path = {'path': [child.state],
                'number': [child.number]}
        while child.parent:
            parent = child.parent
            path['path'].append(parent.state)
            path['number'].append(parent.number)
            child = parent
        path['path'] = self.reverse(path['path'])
        path['number'] = self.reverse(path['number'])
        return path
