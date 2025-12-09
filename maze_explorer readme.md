# Maze Exploration – DFS Backtracking
This project implements full maze exploration using Depth-First Search with backtracking.  
A robot explores each cell of the maze, detects walls, marks visited cells, and automatically returns when no new moves remain.


# The algorithm
- Moves through the maze one cell at a time  
- Uses sensors to detect walls  
- Records wall information in a grid  
- Uses a stack to remember the path  
- Backtracks when it reaches a dead end  
- Guarantees full exploration of all reachable cells


# Maze Representation
Two main grids are used:
maze[x][y] : stores walls around each cell using bit flags.
visited[x][y] : marks whether the robot has visited a cell.


# Movement diections 
dx , dy = {north, east, south, west}


# Wall Detection
Define northBlocked(), eastBlocked(), southBlocked(), westBlocked() functions to decide path.

# move function
Define such that this moves the robot from one cell to it's neighbor cell in the given direction as a parameter.

