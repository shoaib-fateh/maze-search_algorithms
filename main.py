import sys
import time
import pygame
from collections import deque

# --- Node Class ---
# A simple container for a maze cell's state and backtracking info.
class Node:
    def __init__(self, state, parent=None, action=None):
        self.state = state  
        self.parent = parent  
        self.action = action  

# --- QueueFrontier Class (for BFS) ---
# Manages our frontier as a queue.
class QueueFrontier:
    def __init__(self):
        self.frontier = deque()
    
    def add(self, node):
        self.frontier.append(node)
    
    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)
    
    def empty(self):
        return len(self.frontier) == 0
    
    def remove(self):
        if self.empty():
            raise Exception("Frontier is empty")
        return self.frontier.popleft()

# --- Maze Class ---
# Reads the maze from a file and sets up grid, walls, start, and goal.
class Maze:
    def __init__(self, filename):
        with open(filename) as f:
            contents = f.read()

        # Make sure there's exactly one start (A) and one goal (B)
        if contents.count("A") != 1 or contents.count("B") != 1:
            raise Exception("Maze must have exactly one start (A) and one goal (B)")

        self.grid = [list(line) for line in contents.splitlines()]
        self.height = len(self.grid)
        self.width = max(len(row) for row in self.grid)

        # Build the walls grid and mark start/goal positions
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                if self.grid[i][j] == "A":
                    self.start = (i, j)
                    row.append(False)
                elif self.grid[i][j] == "B":
                    self.goal = (i, j)
                    row.append(False)
                elif self.grid[i][j] == " ":
                    row.append(False)
                else:
                    row.append(True)
            self.walls.append(row)

        self.solution = None  # Final solution path will be stored here
        self.explored_nodes = set()
        self.dead_ends = []  # Positions where no further moves are possible

    # Returns valid neighboring moves (up, down, left, right)
    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1)),
        ]
        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

# --- DFS Class ---
# Depth-first search: good old recursion without the recursion (using a stack).
class DFS:
    def __init__(self, maze):
        self.maze = maze

    def solve(self):
        start_node = Node(self.maze.start)
        frontier = [start_node]  # Stack for DFS
        came_from = {self.maze.start: None}
        steps = []  # Order of cell exploration

        while frontier:
            node = frontier.pop()  # LIFO
            state = node.state

            if state == self.maze.goal:
                # Build the solution path by backtracking
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            valid_moves = 0
            for action, neighbor in self.maze.neighbors(state):
                if neighbor not in came_from:
                    frontier.append(Node(neighbor, node, action))
                    came_from[neighbor] = state
                    steps.append(neighbor)
                    valid_moves += 1

            # If no further moves and not at start/goal, mark as dead end
            if valid_moves == 0 and state != self.maze.goal and state != self.maze.start:
                if state not in self.maze.dead_ends:
                    self.maze.dead_ends.append(state)

        return steps

# --- BFS Class ---
# Breadth-first search: exploring layer by layer.
class BFS:
    def __init__(self, maze):
        self.maze = maze

    def solve(self):
        start_node = Node(self.maze.start)
        frontier = QueueFrontier()
        frontier.add(start_node)
        came_from = {self.maze.start: None}
        steps = []

        while not frontier.empty():
            node = frontier.remove()
            state = node.state

            if state == self.maze.goal:
                # Backtrack to form the solution path
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            valid_moves = 0
            for action, neighbor in self.maze.neighbors(state):
                if neighbor not in came_from:
                    frontier.add(Node(neighbor, node, action))
                    came_from[neighbor] = state
                    steps.append(neighbor)
                    valid_moves += 1

            # Mark dead ends when stuck (excluding start and goal)
            if valid_moves == 0 and state != self.maze.goal and state != self.maze.start:
                if state not in self.maze.dead_ends:
                    self.maze.dead_ends.append(state)

        return steps

# --- AStar Class ---
# A* search using Manhattan distance as the heuristic.
class AStar:
    def __init__(self, maze):
        self.maze = maze

    def solve(self):
        def heuristic(a, b):
            # Manhattan distance
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        start_node = Node(self.maze.start)
        frontier = [(0, start_node)]
        came_from = {self.maze.start: None}
        cost_so_far = {self.maze.start: 0}
        steps = []

        while frontier:
            _, node = min(frontier, key=lambda x: x[0])
            frontier.remove((_, node))
            state = node.state

            if state == self.maze.goal:
                # Reconstruct the solution path
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            valid_moves = 0
            for action, neighbor in self.maze.neighbors(state):
                new_cost = cost_so_far[state] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, self.maze.goal)
                    frontier.append((priority, Node(neighbor, node, action)))
                    came_from[neighbor] = state
                    steps.append(neighbor)
                    valid_moves += 1

            # Record dead end if stuck
            if valid_moves == 0 and state != self.maze.goal and state != self.maze.start:
                if state not in self.maze.dead_ends:
                    self.maze.dead_ends.append(state)

        return steps

# --- MazeVisualizer Class ---
# Draws the maze and animates the solving process.
class MazeVisualizer:
    def __init__(self, maze, algorithm="bfs"):
        pygame.init()
        self.maze = maze
        self.cell_size = 15
        self.width = self.maze.width * self.cell_size
        self.height = self.maze.height * self.cell_size

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(f"Maze | {algorithm.upper()} Algorithm")

        # Load images for maze elements
        self.wall_img = pygame.image.load("./blocks/wall.png")
        self.path_img = pygame.image.load("./blocks/path.png")
        self.start_img = pygame.image.load("./blocks/start.png")
        self.goal_img = pygame.image.load("./blocks/goal.png")
        self.visited_img = pygame.image.load("./blocks/visited.png")
        self.solution_img = pygame.image.load("./blocks/solution.png")
        self.end_img = pygame.image.load("./blocks/end.png")

        # Resize images to match our cell dimensions
        self.wall_img = pygame.transform.scale(self.wall_img, (self.cell_size, self.cell_size))
        self.path_img = pygame.transform.scale(self.path_img, (self.cell_size, self.cell_size))
        self.start_img = pygame.transform.scale(self.start_img, (self.cell_size, self.cell_size))
        self.goal_img = pygame.transform.scale(self.goal_img, (self.cell_size, self.cell_size))
        self.visited_img = pygame.transform.scale(self.visited_img, (self.cell_size, self.cell_size))
        self.solution_img = pygame.transform.scale(self.solution_img, (self.cell_size, self.cell_size))
        self.end_img = pygame.transform.scale(self.end_img, (self.cell_size, self.cell_size))

        self.choose_algorithm(algorithm)
        self.run_visualization()

    # Choose which search algorithm to use
    def choose_algorithm(self, algorithm_type):
        if algorithm_type.lower() == "dfs":
            self.pathfinding = DFS(self.maze)
        elif algorithm_type.lower() == "bfs":
            self.pathfinding = BFS(self.maze)
        elif algorithm_type.lower() == "a_star":
            self.pathfinding = AStar(self.maze)
        else:
            raise ValueError("Unknown algorithm: " + algorithm_type)

        # Time the solving process
        start_time = time.time()
        self.steps = self.pathfinding.solve()
        end_time = time.time()
        self.total_time = end_time - start_time

    # Draw the static maze grid
    def draw_maze(self):
        for i in range(self.maze.height):
            for j in range(self.maze.width):
                x = j * self.cell_size
                y = i * self.cell_size
                if self.maze.grid[i][j] == "#":
                    self.screen.blit(self.wall_img, (x, y))
                elif self.maze.grid[i][j] == "A":
                    self.screen.blit(self.start_img, (x, y))
                elif self.maze.grid[i][j] == "B":
                    self.screen.blit(self.goal_img, (x, y))
                else:
                    self.screen.blit(self.path_img, (x, y))

    # Animate the exploration process
    def explore_maze_step_by_step(self):
        for index, state in enumerate(self.steps):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            row, col = state
            if (row, col) == self.maze.goal:
                continue  

            x = col * self.cell_size
            y = row * self.cell_size

            # Use end_img for dead ends, otherwise visited_img
            if state in self.maze.dead_ends:
                self.screen.blit(self.end_img, (x, y))
            else:
                self.screen.blit(self.visited_img, (x, y))
            pygame.display.update()
            time.sleep(0.01)
        self.draw_solution_path()

    # Animate the final solution path
    def draw_solution_path(self):
        if not self.maze.solution:
            return

        for state in self.maze.solution:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            row, col = state
            if (row, col) == self.maze.goal:
                continue  

            x = col * self.cell_size
            y = row * self.cell_size
            self.screen.blit(self.solution_img, (x, y))
            pygame.display.update()
            time.sleep(0.01)

    # Main loop: draw maze, animate exploration, and keep window open.
    def run_visualization(self):
        running = True
        self.draw_maze()
        pygame.display.update()
        self.explore_maze_step_by_step()

        total_steps = len(self.steps)
        solution_length = len(self.maze.solution) if self.maze.solution else 0
        print("\033[92mMaze solved!\033[0m")
        print(f"\033[93mTotal nodes checked: \033[0m{total_steps}")
        print(f"\033[93mSolution length (from start to goal): \033[0m{solution_length}")
        print(f"\033[91mTotal time taken: \033[0m{self.total_time:.4f} seconds")

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit()

# --- Main Entry Point ---
if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("Usage: python main.py ./mazemaps/maze.txt algorithm\nExample: python main.py ./mazemaps/maze.txt bfs")
    maze = Maze(sys.argv[1])
    algorithm = sys.argv[2]  # Options: "dfs", "bfs", or "a_star"
    MazeVisualizer(maze, algorithm)
