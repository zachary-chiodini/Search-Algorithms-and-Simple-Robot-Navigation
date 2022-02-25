import matplotlib.pyplot as plt
from matplotlib.patheffects import Normal, Stroke
from typing import List, Tuple

from problem import State
from tree import Node, Path


class Graph:
    def __init__(self):
        self.graph: List[Tuple[float, float]] = []
        self.coord: List[Tuple[float, float]] = []
        self.treei = 0
        self.gridi = 0

    def grid(self, node: Node, m: int, n: int,
             node_actions: List[Node] = [], rsize=40) -> None:
        """Graphs the path on a grid"""
        # Getting path to state
        path: Path = node.path_to(node)
        path_states: List[State] = path['path']
        path_transformed: List[State] = [[] for _ in range(len(path_states[0]))]
        # Linear transformation of j substates (xj, yj)
        for state in path_states:
            j = 0
            for sub_j in state:
                xj, yj = sub_j[0], n + 1 - sub_j[1]
                path_transformed[j].append((xj, yj))
                j += 1
        plt.clf()  # Clear current figure
        # Robot colors
        color = ['#FF00FF', '#FF8000',
                 '#DD0000', '#FF69B4',
                 '#5E5EFF', '#454545']
        # Plotting the path to state
        j = 0
        for sub_j in path_transformed:
            coords = list(zip(*sub_j))
            plt.plot(*coords, marker='o', color=color[j % 6],
                     alpha=0.5, linewidth=10, markersize=rsize)
            j += 1
        effect = [Stroke(linewidth=5, foreground='white'), Normal()]
        # Plotting available actions
        for node_f in node_actions:
            j = 0
            for sub_j in node_f.state:
                parent = node.state[j]
                xp, yp = parent[0], n + 1 - parent[1]
                xc, yc = sub_j[0], n + 1 - sub_j[1]
                plt.plot([xp, xc], [yp, yc], marker='o',
                         color='#02FF02', alpha=0.5,
                         linewidth=10, markersize=rsize)
                j += 1
        # Plotting each robot's location
        j = 0
        for sub_j in node.state:
            xj, yj = sub_j[0], n + 1 - sub_j[1]
            plt.plot(xj, yj, marker='o', color=color[j % 6],
                     markersize=rsize, path_effects=effect)
            j += 1
        self.gridi += 1
        plt.grid(b=True, color='white', linewidth=5)
        ax = plt.gca()
        ax.set_xticks(
            [i for i in range(1, m + 1)], minor=True)
        ax.set_xticklabels(
            [str(i) for i in range(1, m + 1)], minor=True)
        ax.set_yticks(
            [i for i in range(1, n + 1)], minor=True)
        ax.set_yticklabels(
            [str(i) for i in range(n, 0, -1)], minor=True)
        ax.set_xticks(
            [i + 0.5 for i in range(m + 1)], minor=False)
        ax.set_yticks(
            [i + 0.5 for i in range(n + 1)], minor=False)
        ax.tick_params(color='none', labelcolor='#02FF02',
                       labelsize=20, labeltop=True,
                       labelbottom=False, which='minor')
        ax.tick_params(colors='none', which='major')
        ax.set_facecolor('none')
        ax.set_aspect(1)
        plt.savefig('grid{}.png'.format(self.gridi),
                    dpi='figure', orientation='landscape',
                    transparent=True)
        return None

    def tree(self, node: Node, expanded: List[Node] = []) -> None:
        """Graphs the tree"""
        if node.parent:
            # Retrieving parent coordinate
            xp, yp = self.coord[node.parent.number]
            # Generating child coordinates
            lineage: int = node.lineage(node)
            siblings: int = node.parent.children
            yc = yp - 1
            xc = node.sibling + 0.5 - siblings / 2 \
                if siblings % 2 == 0 \
                else node.sibling - siblings // 2
            xc = xp + xc / lineage
        else:
            xc, yc = 0, 0
            xp, yp = 0, 0
        self.coord.append((xc, yc))
        # Retrieving path coordinates
        path: Path = node.path_to(node)
        lbls = [str(list(state)) for state in path['path']]
        path_coords = [self.coord[num] for num in path['number']]
        pack = list(zip(*path_coords))
        # Plotting the tree
        effect = [Stroke(linewidth=5, foreground='white'), Normal()]
        # Generating branches
        for child in expanded:
            # Retrieving parent coordinate
            xp, yp = self.coord[child.parent.number]
            yci = yp - 1
            # Generating child coordinates
            lineage: int = child.lineage(child)
            siblings: int = child.parent.children
            xci = child.sibling + 0.5 - siblings / 2 \
                if siblings % 2 == 0 \
                else child.sibling - siblings // 2
            xci = xp + xci / lineage
            branch = [(xp, yp), (xci, yci)]
            branch = list(zip(*branch))
            self.graph.extend(branch)
        plt.clf()  # Clear current figure
        plt.plot(*self.graph,
                 color='#02FF02',
                 marker='o',
                 linewidth=2,
                 markersize=10,
                 path_effects=effect)
        # Highlighting path
        plt.plot(*pack,
                 color='#FF00FF',
                 marker='o',
                 linewidth=2,
                 markersize=10,
                 path_effects=effect)
        # Generating text
        for i in range(len(path_coords)):
            xl, yl = path_coords[i]
            plt.text(xl, yl + 0.5,
                     lbls[i],
                     color='white',
                     fontsize=15,
                     bbox=dict(alpha=0.9,
                               facecolor='#454545',
                               edgecolor='none',
                               boxstyle='round'),
                     verticalalignment='center',
                     horizontalalignment='center')
        self.treei += 1
        plt.axis('off')
        plt.grid(b=None)
        plt.savefig('tree{}.png'.format(self.treei),
                    dpi='figure', orientation='landscape',
                    transparent=True)
        return None
