<h1>Search Algorithms & Simple Robot Navigation</h1>
<p style="text-align:justify">
  The purpose of this webpage is to study and gain an understanding of some of the most common search algorithms in AI. 
  These are breadth-first search, depth-first search, greedy search and A* search.
  The algorithms are written in Python and colorful animations are used to show how they work.
  They will be used by robots to make decisions as they navigate a lonely and barren virtual world.
</p>
<p style="text-align:justify">
  This problem of simple robot navigation involves N simple circular robots of radius R on a 2D m by n grid, 
  an initial state specifying the initial positions of the robots and a goal state specifying the final positions of the robots.
  A solution to this problem is a path from the initial to goal state in which no robot exits the grid or smashes into another.
  The aforementioned searching algorithms will be used to find these solutions. 
  Furthermore, the optimality, completeness, time complexity and space complexity of each algorithm will be compared.
</p>
 
<h1>Formalization</h1>
<p style="text-align:justify">
  In order to use a computer to solve a problem, we have to formulate the problem in such a way that a computer will be abe to understand it,
  which often involves mathematics. If you are bored by this, simply skip it and proceed to the animations, where you can
  observe what is explained here.
</p>
<p style="text-align:justify">
  The grid is represented by an m by n matrix in which each element is the x-y coordinate location of each cell in the grid.
</P> 

<p align="center"><img style="width:30%" src="photos/grid.png"/></p>

<p style="text-align:justify">
  Each robot is given a unique integer identifier j. The cell in which the j<span style="font-size:xx-small; vertical-align:super">th</span>
  robot is located is called a substate. The state of N robots is a list of each substate indexed by j in ascending order.
  Each robot j has a set of 9 available actions:
  { up, down, right, left, up-right, up-left, down-right, down-left, idle }.
  The robot chooses 1 of these actions to reach 1 of 9 unique substates.
  N robots provide N unique sets of 9 actions having 9<span style="font-size:xx-small; vertical-align:super">N</span> 
  unique combinations, assuming their movement is unrestricted.
  This provides 9<span style="font-size:xx-small; vertical-align:super">N</span> reachable states. 
  The reachable states of N robots is called the successor set.
</p>

<p align="center"><img style="width:40%" src="photos/definitions.png"/></p>

<p style="text-align:justify">
  Robot radii are a parameter used to calculate collisions and are allowed to be any length. 
  There is a vague concept of velocity in the program. 
  The speed v of the robots must be identical moving horizontally and vertically and √2v when moving diagonally.
  However, transitions from one state to another occur instantaneously. 
  All robots must arrive at their subsequent substates at the same time, but that time is arbitrary.
<p style="text-align:justify">
  The following restrictions on the successor set apply:
  (1) robots move by one grid unit if horizontally or vertically and two grid units if diagonally,
  (2) robots cannot exit the grid, 
  (3) robots cannot collide.
  Restrictions (1) and (2) are straight forward and summarized below.
</p>

<p align="center"><img style="width:70%" src="photos/conditions.png"/></p>

<p style="text-align:justify">
  For restriction (3), simple kinematics can be used to determine whether an action leads to a collision. 
  The derivation of how this is done is summarized below. 
  The time taken to transition between substates is set to 1 second, for convenience.
  If one is not familiar with kinematics, it would be hard to learn from reading this alone.
  To avoid confusion, just enjoy the proceeding animations and observe that the robots indeed do not collide.
</p>

<p align="center"><img src="photos/kinematics.png"/></p>

<h1>Writing the Problem in Python</h1>
<p>
  The problem is defined below in Python given an initial state, a goal state, the radius of the robots R, m and n.
  The diameter of the robots will be set to 90% the length of a grid cell for all examples on this page.
</p>
  
[problem.py](problem.py)
  
<h1>Nodes and Tree Data Structure</h1>
<p style="text-align:justify">
  A searching algorithm typically searches for a solution in what is called a tree.
  Tree structures are composed of nodes. A single node has a single parent node and one or more
  child nodes, for which it is the parent node. The branches extending from parent to child nodes create
  what looks like an upside down tree in which the root node is at the top. The root node is the only node
  that has no parent and represents the grandparent of all nodes in the tree. 
  The depth of the node is the number of parent nodes above it. 
  Sibling nodes are nodes that share the same parent. Leaf nodes are the bottommost nodes, which have no children.
  The path of a node is its lineage to the root node.
</p>
<p style="text-align:justify">
  For this problem, each node represents a state.
  Child nodes represent the states reachable from the parent node.
  The branch connecting the parent node to the child node represents the action taken to reach the child state. 
  The children of a node are generated by the successor. This is termed "expanding" the node.
  The initial state is the root node, and the goal state is a leaf node.
  A solution is the path from the root node to the node containing the goal state.
</p>
<h1>Writing the Tree in Python</h1>
<p style="text-align:justify">
  This script defines a node which is used to create the tree.
  The paremeter number uniquely identifies a node. This is necessary when graphing the 
  tree, because the state itself cannot be used to uniquely identify the node.
  The parameter sibling is the number of the node in relation to its siblings. This is used to determine the horizontal position of 
  the node in the tree when graphing. The function lineage generates a factor
  used to constrict the spread of lower branches in the tree when they are graphed so they do not overlap. 
  The path cost will be explained later.
</p>

[tree.py](tree.py)

<h1>Graphing and Animating the Search Process</h1>
<p style="text-align:justify">
  Graphing and animating the search process is used to visualize how the algorithms work. 
  This class generates a png of the search tree and the current path of the robot on the grid, given a node.
  The tree and path are regenerated with each node processed. The png files are turned into png animations outside of Python.
</p>
  
[graph.py](graph.py)

<h1>Breadth-First Search</h1>
<p style="text-align:justify">
  This algorithm is used to explore the tree by expanding all of the nodes at the present depth prior to moving on to the next depth.
  In other words, breadth-first search explores paths of length 1 first, then all those of length 2, and so on.
  Therefore, if a solution exists, breadth-first search will find the shallowest goal state first, making it 
  complete and optimal. However, beadth-first search has an exponential time and space complexity bound,
  which means it is only practical for simple problems.
</p>
<h1>Breadth-First Search in Python</h1>
<p style="text-align:justify">
  The algorithm is written using the classes above. A list containing explored nodes is used
  to prevent the aglorithm from exploring previously visited states.
  The frontier contains all of the nodes to be expanded as a first-in-first-out (FIFO) queue.
</p>

[bfs.py](bfs.py)

<h1>Examples</h1>
<p style="text-align:justify">
  With only 1 robot on a 4 by 4 grid with an initial state [ ( 1, 1 ) ] and a goal state [ ( 4, 4 ) ],
  breadth-first search finds the path below.
  (This site uses animated png files to illustrate how the algorithms work. 
  If you are using a browser that does not support animated png files, please switch to one that does.)
</p>

<p align="center"><img src="animations/BFS/solution1.png"/></p>

<p style="text-align:justify">
  The search process and path is animated below.
  The violet portion of the search tree shows the current path.
  The current path is also animated on the grid to the right of the tree.
  Available actions are highlighted in green on the grid.
  The tree expands 30 nodes before finding a solution. 
  This is the shortest path to the goal state and therefore the optimal solution.
</p>
<p style="text-align:justify">
  Some nodes produce less children because available actions are reduced by encountering the grid boundary 
  and by preventing revisits to previously explored states.
</p>
<p align="center"><img src="animations/BFS/BFS_1r.png"/><img src="animations/BFS/BFS_1r_grid.png"/></p>
<p style="text-align:justify">
  With 2 robots on a 2 by 3 grid with an initial state [ ( 1, 1 ), ( 2, 1 ) ] and a goal state [ ( 2, 3 ), ( 1, 3 ) ],
  breadth-first search finds the path below.
</p>
<p align="center"><img src="animations/BFS/solution2.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below.
  On the grid to the right of the tree, the path of robot 1 is
  highlighted in violet and the path of robot 2 is highlighted in orange. However, the solution is a single path
  containing both robots. The tree expands 79 nodes before finding an optimal solution.
  You can see how breadth-first search exhausts every possible path of a certain length before moving on to the next, until
  it reaches the goal state. 
  Due to the exponential time and space complexity of breadth-fist search, more complicated examples cannot be explored.
</p>
<p align="center"><img src="animations/BFS/BFS_2x3_2r.png"/><img src="animations/BFS/BFS_2x3_2r_grid.png"/></p>
<h1>Depth-First Search</h1>
<p style="text-align:justify">
  This alogirthm explores the deepest level of the tree first. When the search hits a leaf node, it backtracks up the tree and
  expands a shallower level node to the deepest level, and so on. Depth-first search has an exponential time complexity bound but
  may be faster than breadth-first search if the correct node is chosen at the start of the search, or if there are many solutions
  in the tree. Its space complexity is the product of the number of branches and depth of the tree. 
  Depth-first search is not complete, because it may not find a solution if the node it chooses to expand leads 
  to an infinite or very deep leaf node. 
  It is also not optimal, because it does not guarantee that the shallowest goal state will be found.
</p>
<h1>Depth-First Search in Python</h1>
<p style="text-align:justify">
  The script is identical to breadth-fist search except the frontier is a last-in-first-out (LIFO) queue.
</p>
<table align="center">
<tr><td>
<div style="height:300px; width:750px; overflow:auto; font-family:courier">
<table class="table"><tr><td><div class="linenodiv" style="background-color: #454545; padding-right: 10px"><pre style="line-height: 125%"> 1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30</pre></div></td><td class="code"><div style="background: #002240"><pre style="line-height: 125%"><span></span><span style="color: #FF8000">def</span> <span style="color: #5E5EFF">depth_first_search</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> ) :
  <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span> <span style="color: #FF8000">=</span> <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">init</span> ), <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">goal</span> )
  <span style="color: #FFFFFF">graph</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Graph</span>()
  <span style="color: #FFFFFF">problem</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Problem</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
  <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># node identifier for graph</span>
  <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># sibling identifier for graph</span>
  <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">=</span> [ <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">init</span> ) ]
  <span style="color: #FFFFFF">explored</span> <span style="color: #FF8000">=</span> []
  <span style="color: #FF8000">while</span> <span style="color: #FF69B4">True</span> :
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">==</span> [] : <span style="color: #FF8000">return</span> <span style="color: #02FF02">&#39;Failure&#39;</span>
      <span style="color: #FFFFFF">node</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">pop</span>( <span style="color: #FF8000">-</span><span style="color: #FF00FF">1</span> ) <span style="color: #DD0000"># LIFO queue</span>
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">goal_test</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">path</span>( <span style="color: #FFFFFF">node</span> )[ <span style="color: #02FF02">&#39;path&#39;</span> ]
      <span style="color: #FFFFFF">explored</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> )
      <span style="color: #FFFFFF">children</span> <span style="color: #FF8000">=</span> []
      <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">state</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">successor</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FFFFFF">child</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">parent</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node</span> )
          <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> <span style="color: #FF8000">not</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">explored</span> :
              <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">children</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">number</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node_id</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">sibling</span>
              <span style="color: #FFFFFF">children</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
      <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">children</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span>, <span style="color: #FFFFFF">children</span> )
</pre></div>
</td></tr></table>
</div>
</td></tr>
</table>
<h1>Examples</h1>
<p style="text-align:justify">
  With only 1 robot on a 4 by 4 grid with an initial state [ ( 1, 1 ) ] and a goal state [ ( 4, 4 ) ],
  depth-first search finds the path below.
</p>
<p align="center"><img src="animations/DFS/solution1.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. The search expands 10 nodes before finding a solution.
  Although depth-first search found a solution after expanded less nodes, the solution is far from optimal.
</p>
<p align="center"><img src="animations/DFS/DFS_1r.png"/><img src="animations/DFS/DFS_1r_grid.png"/></p>
<p style="text-align:justify">
  With 2 robots on a 2 by 3 grid with an initial state [ ( 1, 1 ), ( 2, 1 ) ] and a goal state [ ( 2, 3 ), ( 1, 3 ) ],
  depth-first search finds the path.
</p>
<p align="center"><img src="animations/DFS/solution2.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. The search expands 34 nodes before finding a solution. 
  You can see how depth-first search dives into the first node it encounters until a solution or dead end is found. 
  This leads to an aimless path in which the robots seem to randomly stumble onto the goal state. 
  In the example, depth-first search actually encounters a dead end and backtracks up the tree before settling on the solution.
</p>
<p align="center"><img src="animations/DFS/DFS_2x3_2r.png"/><img src="animations/DFS/DFS_2x3_2r_grid.png"/></p>
<h1>Greedy Search</h1>
<p style="text-align:justify">
  This algorithm chooses nodes based on their heuristic value in ascending order.
  The heuristic value of a node is an estimate of the "cost" to reach the goal from that node.
  In this problem, the heuristic value of a node or state is the sum of the straight line distances from each substate
  to their respective goal substates. Therefore, the goal state has a heuristic value of zero, 
  which is a condition all heuristic functions must have.
  This algorithm finds solutions quickly, but does not always find the optimal solution. Like depth-first search, the algorithm
  tends to follow a single path all the way to the goal and backtracks up the tree and expands shallower nodes when it hits a
  dead end. Additionally, like depth-first search, greedy search is not complete or optimal for the same reasons. 
  Greedy search has, in the worst case, an exponential space and time complexity bound, but with a good heuristic function,
  these can be significantly reduced.
</p>
<h1>Greedy Search in Python</h1>
<p style="text-align:justify">
  The script is identical to breadth-first and depth-fist search except the frontier is not a FIFO or LIFO queue but a priority
  queue in which nodes are prioritized by their heuristic value in ascending order.
</p>
<table align="center">
<tr><td>
<div style="height:300px; width:750px; overflow:auto; font-family:courier">
<table class="table"><tr><td><div class="linenodiv" style="background-color: #454545; padding-right: 10px"><pre style="line-height: 125%"> 1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58</pre></div></td><td class="code"><div style="background: #002240"><pre style="line-height: 125%"><span></span><span style="color: #FF8000">def</span> <span style="color: #5E5EFF">greedy_search</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> ) :
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">magnitude</span>( <span style="color: #FFFFFF">v</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; Magnitude of vector v &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> ( <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">0</span> ]<span style="color: #FF8000">**</span><span style="color: #FF00FF">2</span> <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">1</span> ]<span style="color: #FF8000">**</span><span style="color: #FF00FF">2</span> )<span style="color: #FF8000">**</span><span style="color: #FF00FF">0.5</span>
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">difference</span>( <span style="color: #FFFFFF">sub_f</span>, <span style="color: #FFFFFF">sub_i</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Vector difference between substate f and i </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">sub_f</span>[ <span style="color: #FF00FF">0</span> ] <span style="color: #FF8000">-</span> <span style="color: #FFFFFF">sub_i</span>[ <span style="color: #FF00FF">0</span> ], <span style="color: #FFFFFF">sub_f</span>[ <span style="color: #FF00FF">1</span> ] <span style="color: #FF8000">-</span> <span style="color: #FFFFFF">sub_i</span>[ <span style="color: #FF00FF">1</span> ]
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">distance</span>( <span style="color: #FFFFFF">sub_i</span>, <span style="color: #FFFFFF">sub_f</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Straight line distance from substate i to f </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">difference</span>( <span style="color: #FFFFFF">sub_f</span>, <span style="color: #FFFFFF">sub_i</span> ) )
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">heuristic</span>( <span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">goal</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Sum of estimated distance each robot j has to</span>
<span style="color: #02FF02">        travel from substate j to the goal substate j</span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FF69B4">sum</span>( <span style="color: #FFFFFF">distance</span>( <span style="color: #FFFFFF">state</span>[ <span style="color: #FFFFFF">j</span> ], <span style="color: #FFFFFF">goal</span>[ <span style="color: #FFFFFF">j</span> ] ) 
                  <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">j</span> <span style="color: #FF8000">in</span> <span style="color: #FF69B4">range</span>( <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">robots</span> ) )
  <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span> <span style="color: #FF8000">=</span> <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">init</span> ), <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">goal</span> )
  <span style="color: #FFFFFF">graph</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Graph</span>()
  <span style="color: #FFFFFF">problem</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Problem</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
  <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># node identifier for graph</span>
  <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># sibling identifier for graph</span>
  <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">=</span> { <span style="color: #FFFFFF">heuristic</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span> ) : [ <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">init</span> ) ] }
  <span style="color: #FFFFFF">explored</span> <span style="color: #FF8000">=</span> []
  <span style="color: #FF8000">while</span> <span style="color: #FF69B4">True</span> :
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">==</span> [] : <span style="color: #FF8000">return</span> <span style="color: #02FF02">&#39;Failure&#39;</span>
      <span style="color: #DD0000"># Priority queue</span>
      <span style="color: #FFFFFF">slct</span> <span style="color: #FF8000">=</span> <span style="color: #FF69B4">min</span>( <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">keys</span>() )
      <span style="color: #FFFFFF">node</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">slct</span> ]<span style="color: #FF8000">.</span><span style="color: #FFFFFF">pop</span>()
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">slct</span> ] <span style="color: #FF8000">==</span> [] :
          <span style="color: #FF8000">del</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">slct</span> ]
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">goal_test</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">path</span>( <span style="color: #FFFFFF">node</span> )[ <span style="color: #02FF02">&#39;path&#39;</span> ]
      <span style="color: #FFFFFF">explored</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> )
      <span style="color: #FFFFFF">children</span> <span style="color: #FF8000">=</span> []
      <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">state</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">successor</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FFFFFF">child</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">parent</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node</span> )
          <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> <span style="color: #FF8000">not</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">explored</span> :
              <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">children</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">number</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node_id</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">sibling</span>
              <span style="color: #FFFFFF">children</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FFFFFF">h</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">heuristic</span>( <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">goal</span> )
              <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">h</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">keys</span>() :
                  <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">h</span> ]<span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FF8000">else</span> : 
                  <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">h</span> ] <span style="color: #FF8000">=</span> [ <span style="color: #FFFFFF">child</span> ]
              <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
      <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">children</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span>, <span style="color: #FFFFFF">children</span> )
</pre></div>
</td></tr></table>
</div>
</td></tr>
</table>
<h1>Examples</h1>
<p style="text-align:justify">
  With only 1 robot on a 4 by 4 grid with an initial state [ ( 1, 1 ) ] and a goal state [ ( 4, 4 ) ],
  greedy search finds the path below.
</p>
<p align="center"><img src="animations/BFS/solution1.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. The algorithm finds the optimal solution after expanding only 3 nodes.
</p>
<p align="center"><img src="animations/greedy/greedy_1r.png"/><img src="animations/greedy/greedy_1r_grid.png"/></p>
<p style="text-align:justify">
  With 2 robots on a 2 by 3 grid with an initial state [ ( 1, 1 ), ( 2, 1 ) ] and a goal state [ ( 2, 3 ), ( 1, 3 ) ],
  greedy search finds the path below.
  <p align="center"><img src="animations/greedy/solution2.png"/></p>
</p>
<p style="text-align:justify">
  The search process and path is animated below. Greedy search finds a solution after expanding only 5 nodes, but it is suboptimal.
  The two robots rush toward their goal substates, but have to turn around to avoid a collision. 
  The optimal solution involves one of the robots remaining idle for the first move (see breadth-first search),
  which greedy search will never find (it's too greedy!).
  Though greedy search often significantly reduces the time taken to find a solution, the solution is often suboptimal.
</p>
 <p align="center"><img src="animations/greedy/greedy_2x3_2r.png"/><img src="animations/greedy/greedy_2x3_2r_grid.png"/></p>
<p style="text-align:justify">
  Due to the speed of this algorithm, more complex examples can be explored.
  With 2 robots on a 7 by 7 grid with an initial state [ ( 2, 2 ), ( 6, 6 ) ] and a goal state [ ( 6, 6 ), ( 2, 2 ) ],
  greedy search finds the path below.
</p>
<p align="center"><img src="animations/greedy/solution_7x7_2r.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. Greedy search finds a solution after expanding 6 nodes.
  The depth of this solution is also 6. The shallowest goal state exists at a depth of 5, in which the two 
  robots anticipate the collision in the center of the grid and get out of each others way at the same time.
  Greedy search will never find this solution. If only these two robots could work together instead of being greedy.
  Will they ever learn?
<p style="text-align:justify">
  By separating the 2 robots and increasing the dimensions of the grid
  so that the number of actions available to each robot is not restricted, as we have done in this example,
  the problem becomes impossible to solve using breadth-first or depth-first search.
  The reason is the following: The root node expands into 80 child nodes, or 9<span style="font-size:xx-small; vertical-align:super">2</span> 
  minus the state in which both robots are idle. Each of these 80 nodes can be expanded into roughly 80 more child nodes,
  which gives about 80<span style="font-size:xx-small; vertical-align:super">2</span> total.
  The number of nodes increases approximately exponentially with depth, 
  like 80<span style="font-size:xx-small; vertical-align:super">d</span>, where d is the depth.
  The shallowest goal state is found at a depth of 5, which means breadth-first search would have to explore (at worst) roughly 
  80<span style="font-size:xx-small; vertical-align:super">5</span>, or 3 billion nodes, before finding a solution.
  Comparatively, depth-first search would meander aimlessly down a very deep and random path.
</p>
<p style="text-align:justify">
  The maximum depth of the tree is a path in which one of the robots traverses the full area of the grid
  each time the other robot moves to also traverse the full area of the grid, or
  ( n<span style="font-size:xx-small; vertical-align:super">2</span> - 1 )<span style="font-size:xx-small; vertical-align:super">2</span>
  + n<span style="font-size:xx-small; vertical-align:super">2</span> - 1 = n<span style="font-size:xx-small; vertical-align:super">4</span> - n<span style="font-size:xx-small; vertical-align:super">2</span>,
  for m = n, which gives a maximum depth of 2,352 for m = n = 7.
  That means the search tree in its entirety consists of about 80<span style="font-size:xx-small; vertical-align:super">2352</span> nodes.
  Try plugging that into a calculator. That's more than the number of atoms that exist in the observable universe, by more than just a lot!
  Imagine traversing a depth of 2352 in this massive tree. 
  Who would have thought a simple program and a simple problem would produce such an astounding result.
  We should be happy that any solution was found, but we can do better.
</p>
<p align="center"><img src="animations/greedy/greedy_7x7_2r.png"/><img src="animations/greedy/greedy_7x7_2r_grid.png"/></p>
<h1>A* Search</h1>
<p style="text-align:justify">
  This algorithm chooses nodes based on the sum of their heuristic value and path cost in ascending order.
  The path cost is the sum of all step costs in the current path. 
  The step cost is the cost of moving from a parent state to a child state.
  For a typical path finding problem with one robot, the path cost is the actual distance the robot has traveled thus
  far along the path and the hueristic is the estimated remaining distance.
  This way, the search algorithm should find the optimal solution with a priority queue while "pruning" the search tree
  of paths that are predicted to be suboptimal.
  A* search is optimal and complete, as long as the heuristic function is admissable and consistent.
  An admissible heuristic function is one that never overestimates the cost to reach the goal.
  If the heuristic value of a parent node is not greater than the step cost plus the heuristic value of its child node,
  the heuristic function is consistent. Like greedy search, A* search has an exponential space and time complexity bound, 
  but with a good heuristic and path cost function, these can be significantly reduced.
</p>
<h1>A* Search in Python</h1>
<p style="text-align:justify">
  The script is identical to greedy search except the priority queue is prioritized by the heuristic value plus the path cost
  of the nodes in ascending order. The heuristic function is the same as in greedy search but termed the h score.
  The path cost function is termed the g score. The sum of the heuristic value and the path cost is termed the f score.
</p>
<p style="text-align:justify">
  This problem is not a typical path finding problem. The description above fails with multiple robots. The reason is that the path cost
  plus the heuristic value does not consider solutions having idle states as suboptimal. 
  The optimal solution here is the quickest solution, not necessarily the solution in which the robots travel the least distance.
  Careful consideration went into designing the path cost function so that all robots will be directed toward their goal substates
  while the admissibility and consistency of the heuristic function is preserved.
</p>
<p style="text-align:justify">
  The path cost function gives actions directed toward the goal state a lower step cost.
  For robot j moving from a parent to a child substate, the step cost is the striaght line distance from the parent substate
  to the child substate plus the magnitude of the difference of two vectors.
  The vector termed the hueristic vector is directed from the goal substate to the child substate with a magnitude equal to 
  the heuristic value of the child substate. The vector termed the step cost vector is directed from the child substate to 
  the parent substate with a magnitude also equal to the heuristic value of the child substate.
  If the child substate is idle, or equal to the parent substate, 
  the step cost vector is made opposite to the hueristic vector.
  The magnitude of the difference of these two vectors is zero if the robot moves directly toward its goal substate
  and maximum when the robot is idle or moves directly opposite its goal substate.
  The straight line distance from the parent to child substate is added to provide consistency.
  The total step cost for the state is the sum of the step costs of all substates.
  In this way, all N robots are directed toward their goal substates and an optimal solution will be found.
</p>
<table align="center">
<tr><td>
<div style="height:300px; width:750px; overflow:auto; font-family:courier">
<table class="table"><tr><td><div class="linenodiv" style="background-color: #454545; padding-right: 10px"><pre style="line-height: 125%">  1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
75
76
77
78
79
80
81
82
83
84
85
86
87
88
89
90
91
92
93
94
95
96
97
98
99
100
101
102
103
104</pre></div></td><td class="code"><div style="background: #002240"><pre style="line-height: 125%"><span></span><span style="color: #FF8000">def</span> <span style="color: #5E5EFF">astar_search</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> ) :
  <span style="color: #02FF02">&#39;&#39;&#39; Modified A* search &#39;&#39;&#39;</span>
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">magnitude</span>( <span style="color: #FFFFFF">v</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; Magnitude of vector v &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> ( <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">0</span> ]<span style="color: #FF8000">**</span><span style="color: #FF00FF">2</span> <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">1</span> ]<span style="color: #FF8000">**</span><span style="color: #FF00FF">2</span> )<span style="color: #FF8000">**</span><span style="color: #FF00FF">0.5</span>
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">addition</span>( <span style="color: #FFFFFF">sub_j</span>, <span style="color: #FFFFFF">sub_k</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Vector addition between substate j and k </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">sub_j</span>[ <span style="color: #FF00FF">0</span> ] <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">sub_k</span>[ <span style="color: #FF00FF">0</span> ], <span style="color: #FFFFFF">sub_k</span>[ <span style="color: #FF00FF">1</span> ] <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">sub_k</span>[ <span style="color: #FF00FF">1</span> ]
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">difference</span>( <span style="color: #FFFFFF">sub_f</span>, <span style="color: #FFFFFF">sub_i</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Vector difference between substate f and i </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">sub_f</span>[ <span style="color: #FF00FF">0</span> ] <span style="color: #FF8000">-</span> <span style="color: #FFFFFF">sub_i</span>[ <span style="color: #FF00FF">0</span> ], <span style="color: #FFFFFF">sub_f</span>[ <span style="color: #FF00FF">1</span> ] <span style="color: #FF8000">-</span> <span style="color: #FFFFFF">sub_i</span>[ <span style="color: #FF00FF">1</span> ]
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">distance</span>( <span style="color: #FFFFFF">sub_i</span>, <span style="color: #FFFFFF">sub_f</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Straight line distance from substate i to f </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">difference</span>( <span style="color: #FFFFFF">sub_f</span>, <span style="color: #FFFFFF">sub_i</span> ) )
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">set_mag</span>( <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">v</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Change the mangitude of vector v to the value m </span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FFFFFF">mag_v</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">v</span> )
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">mag_v</span> :
          <span style="color: #FFFFFF">nx</span>, <span style="color: #FFFFFF">ny</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">0</span> ] <span style="color: #FF8000">/</span> <span style="color: #FFFFFF">mag_v</span>, <span style="color: #FFFFFF">v</span>[ <span style="color: #FF00FF">1</span> ] <span style="color: #FF8000">/</span> <span style="color: #FFFFFF">mag_v</span>
          <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">m</span><span style="color: #FF8000">*</span><span style="color: #FFFFFF">nx</span>, <span style="color: #FFFFFF">m</span><span style="color: #FF8000">*</span><span style="color: #FFFFFF">ny</span>
      <span style="color: #FF8000">else</span> : 
          <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">v</span>
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">h_score</span>( <span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">goal</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; </span>
<span style="color: #02FF02">        Sum of estimated distance each robot j has to</span>
<span style="color: #02FF02">        travel from substate j to the goal substate j</span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FF69B4">sum</span>( <span style="color: #FFFFFF">distance</span>( <span style="color: #FFFFFF">state</span>[ <span style="color: #FFFFFF">j</span> ], <span style="color: #FFFFFF">goal</span>[ <span style="color: #FFFFFF">j</span> ] ) 
                  <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">j</span> <span style="color: #FF8000">in</span> <span style="color: #FF69B4">range</span>( <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">robots</span> ) )
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">g_score</span>( <span style="color: #FFFFFF">child</span>, <span style="color: #FFFFFF">goal</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39;</span>
<span style="color: #02FF02">        The g score is the path cost function, which is</span>
<span style="color: #02FF02">        the sum of all step costs along the current path.</span>
<span style="color: #02FF02">        The step cost for a child node is the sum of the </span>
<span style="color: #02FF02">        striaght line distance from each parent substate </span>
<span style="color: #02FF02">        to their child substate plus the magnitude of the </span>
<span style="color: #02FF02">        difference of two vectors that serve to increase </span>
<span style="color: #02FF02">        the cost of steps that are idle or directed away </span>
<span style="color: #02FF02">        from the goal.</span>
<span style="color: #02FF02">        &#39;&#39;&#39;</span>
      <span style="color: #FFFFFF">stepcost</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
      <span style="color: #FFFFFF">distance</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
      <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">j</span> <span style="color: #FF8000">in</span> <span style="color: #FF69B4">range</span>( <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">robots</span> ) :
          <span style="color: #DD0000"># position vector relative to goal substate (hueristic vector)</span>
          <span style="color: #FFFFFF">h_vector</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">difference</span>( <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span>[ <span style="color: #FFFFFF">j</span> ], <span style="color: #FFFFFF">goal</span>[ <span style="color: #FFFFFF">j</span> ] )
          <span style="color: #DD0000"># vector pointing from child to parent substate (step cost vector)</span>
          <span style="color: #FFFFFF">s_vector</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">difference</span>( <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">parent</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span>[ <span style="color: #FFFFFF">j</span> ], <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span>[ <span style="color: #FFFFFF">j</span> ] )
          <span style="color: #FFFFFF">distance</span> <span style="color: #FF8000">+=</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">s_vector</span> )
          <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">s_vector</span> ) <span style="color: #FF8000">==</span> <span style="color: #FF00FF">0</span> :
              <span style="color: #FFFFFF">s_vector</span> <span style="color: #FF8000">=</span> <span style="color: #FF8000">-</span><span style="color: #FFFFFF">h_vector</span>[ <span style="color: #FF00FF">0</span> ], <span style="color: #FF8000">-</span><span style="color: #FFFFFF">h_vector</span>[ <span style="color: #FF00FF">1</span> ]
          <span style="color: #FF8000">else</span> :
              <span style="color: #FFFFFF">s_vector</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">set_mag</span>( <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">h_vector</span> ), <span style="color: #FFFFFF">s_vector</span> )
          <span style="color: #FFFFFF">stepcost</span> <span style="color: #FF8000">+=</span> <span style="color: #FFFFFF">magnitude</span>( <span style="color: #FFFFFF">difference</span>( <span style="color: #FFFFFF">h_vector</span>, <span style="color: #FFFFFF">s_vector</span> ) )
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">parent</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">pathcost</span> <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">stepcost</span> <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">distance</span>
  <span style="color: #FF8000">def</span> <span style="color: #5E5EFF">f_score</span>( <span style="color: #FFFFFF">child</span>, <span style="color: #FFFFFF">goal</span> ) :
      <span style="color: #02FF02">&#39;&#39;&#39; Total of g_score plus h_score &#39;&#39;&#39;</span>
      <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">pathcost</span> <span style="color: #FF8000">+</span> <span style="color: #FFFFFF">h_score</span>( <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">goal</span> )
  <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span> <span style="color: #FF8000">=</span> <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">init</span> ), <span style="color: #FF69B4">tuple</span>( <span style="color: #FFFFFF">goal</span> )
  <span style="color: #FFFFFF">graph</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Graph</span>()
  <span style="color: #FFFFFF">problem</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Problem</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span>, <span style="color: #FFFFFF">r</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
  <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># node identifier for graph</span>
  <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span> <span style="color: #DD0000"># sibling identifier for graph</span>
  <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">=</span> { <span style="color: #FFFFFF">h_score</span>( <span style="color: #FFFFFF">init</span>, <span style="color: #FFFFFF">goal</span> ) : [ <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">init</span> ) ] }
  <span style="color: #FFFFFF">explored</span> <span style="color: #FF8000">=</span> []
  <span style="color: #FFFFFF">i</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
  <span style="color: #FF8000">while</span> <span style="color: #FF69B4">True</span> :
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">frontier</span> <span style="color: #FF8000">==</span> [] : <span style="color: #FF8000">return</span> <span style="color: #02FF02">&#39;Failure&#39;</span>
      <span style="color: #DD0000"># Priority queue</span>
      <span style="color: #FFFFFF">f_pa</span> <span style="color: #FF8000">=</span> <span style="color: #FF69B4">min</span>( <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">keys</span>() )
      <span style="color: #FFFFFF">node</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">f_pa</span> ]<span style="color: #FF8000">.</span><span style="color: #FFFFFF">pop</span>()
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">f_pa</span> ] <span style="color: #FF8000">==</span> [] : 
          <span style="color: #FF8000">del</span> <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">f_pa</span> ]
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span> )
      <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">goal_test</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FF8000">return</span> <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">path</span>( <span style="color: #FFFFFF">node</span> )[ <span style="color: #02FF02">&#39;path&#39;</span> ]
      <span style="color: #FFFFFF">explored</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> )
      <span style="color: #FFFFFF">children</span> <span style="color: #FF8000">=</span> []
      <span style="color: #FF8000">for</span> <span style="color: #FFFFFF">state</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">problem</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">successor</span>( <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> ) :
          <span style="color: #FFFFFF">child</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">Node</span>( <span style="color: #FFFFFF">state</span>, <span style="color: #FFFFFF">parent</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node</span> )
          <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">pathcost</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">g_score</span>( <span style="color: #FFFFFF">child</span>, <span style="color: #FFFFFF">goal</span> )
          <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">state</span> <span style="color: #FF8000">not</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">explored</span> :
              <span style="color: #FFFFFF">node_id</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">node</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">children</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">number</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">node_id</span>
              <span style="color: #FFFFFF">child</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">sibling</span>
              <span style="color: #FFFFFF">children</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FFFFFF">f</span> <span style="color: #FF8000">=</span> <span style="color: #FFFFFF">f_score</span>( <span style="color: #FFFFFF">child</span>, <span style="color: #FFFFFF">goal</span> )
              <span style="color: #FF8000">if</span> <span style="color: #FFFFFF">f</span> <span style="color: #FF8000">in</span> <span style="color: #FFFFFF">frontier</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">keys</span>() :
                  <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">f</span> ]<span style="color: #FF8000">.</span><span style="color: #FFFFFF">append</span>( <span style="color: #FFFFFF">child</span> )
              <span style="color: #FF8000">else</span> : 
                  <span style="color: #FFFFFF">frontier</span>[ <span style="color: #FFFFFF">f</span> ] <span style="color: #FF8000">=</span> [ <span style="color: #FFFFFF">child</span> ]
              <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">+=</span> <span style="color: #FF00FF">1</span>
      <span style="color: #FFFFFF">sibling</span> <span style="color: #FF8000">=</span> <span style="color: #FF00FF">0</span>
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">tree</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">children</span> )
      <span style="color: #FFFFFF">graph</span><span style="color: #FF8000">.</span><span style="color: #FFFFFF">grid</span>( <span style="color: #FFFFFF">node</span>, <span style="color: #FFFFFF">m</span>, <span style="color: #FFFFFF">n</span>, <span style="color: #FFFFFF">children</span> )
</pre></div>
</td></tr></table>
</div>
  </td></tr>
  </table>
<h1>Examples</h1>
<p style="text-align:justify">
  With only 1 robot on a 4 by 4 grid with an initial state [ ( 1, 1 ) ] and a goal state [ ( 4, 4 ) ],
  the modified A* search finds the path below.
</p>
<p align="center"><img src="animations/BFS/solution1.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. The algorithm finds the optimal solution after expanding only 3 nodes.
  The search is identical to greedy search when using a single robot.
</p>
<p align="center"><img src="animations/greedy/greedy_1r.png"/><img src="animations/greedy/greedy_1r_grid.png"/></p>
<p style="text-align:justify">
  With 2 robots on a 2 by 3 grid with an initial state [ ( 1, 1 ), ( 2, 1 ) ] and a goal state [ ( 2, 3 ), ( 1, 3 ) ],
  the modified A* search finds the path below.
  <p align="center"><img src="animations/astar/solution2.png"/></p>
</p>
<p style="text-align:justify">
  The search process and path is animated below. The modified A* search finds a solution after expanding 6 nodes, and it is optimal.
  This is a significant improvement from the 79 nodes breadth-first search took to find an optimal solution. 
  This search process is unique. Like greedy search, the robots initially rush toward the goal state, but unlike greedy search,
  once they realize something is obstructing their way, they backtrack up the tree and try a new path.
  They are greedy in that they want to reach the goal state as quickly as possible but not so greedy that they can't take 
  a step back from their initial impulse and reconsider the best coarse of action.
</p>
 <p align="center"><img src="animations/astar/astar_2x3_2r.png"/><img src="animations/astar/astar_2x3_2r_grid.png"/></p>
<p style="text-align:justify">
  With 2 robots on a 7 by 7 grid with an initial state [ ( 2, 2 ), ( 6, 6 ) ] and a goal state [ ( 6, 6 ), ( 2, 2 ) ],
  the modified A* search finds the path below.
</p>
<p align="center"><img src="animations/astar/solution_7x7_2r.png"/></p>
<p style="text-align:justify">
  The search process and path is animated below. 
  The modified A* search finds a solution after expanding 55 nodes.
  This is significantly more than the 6 greedy search took to find a solution, but
  in a world of around 80<span style="font-size:xx-small; vertical-align:super">2352</span> options,
  browsing through just 55 to find an optimal one seems rather efficient.
  Initially, the robots rush to the goal again and almost make the same mistake as their greedy counterparts,
  but instead of pushing eachother aside, they try to work together.
  It appears as though they begin to have some kind of nonverbal debate once they encounter.
  It's as though they are attempting to reach a consensus on the best coarse of action.
  Our robots have matured and become productive citizens of their simple but surprisingly intricate 2D world.
  You can see how our A* search does not need to exhaust all possible paths at shallow depths to find the shallowest
  goal state. It pruned 67 of the 80 branches at the first depth and thousands from the second depth and so on.
  This eliminated billions of nodes from the search, which were predicted to lead to suboptimal solutions by the f score.
</p>
<p align="center"><img src="animations/astar/astar_7x7_2r.png"/><img src="animations/astar/astar_7x7_2r_grid.png"/></p>
<h1>Conclusion</h1>
<p style="text-align:justify">
  Unfortunately, problems involving many robots spaced far apart are still infeasible.
  The time and space complexity of the modified A* search will explode during a many robot encounter.
  Even if a suboptimal solution is adequate, due to the size of the successor set 9<span style="font-size:xx-small; vertical-align:super">N</span>,
  the time and space complexity will explode for large N, no matter how quick the search algorithm is.
  This problem is not actually solvable for arbitrary N, m and n values.
  It was invented as a learning tool to demonstrate how these algorithms work, 
  which are fundamental to AI and computer science. 
  Methods to further reduce the time and space complexity of the search processes and other algorithms or methods
  that could be used to solve this problem more efficiently are beyond the scope of this page.
</p>
