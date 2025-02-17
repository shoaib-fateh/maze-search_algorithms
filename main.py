import sys
import time
import pygame
from collections import deque

# ==============================
# Node class
# ==============================
class Node:
    def __init__(self, state, parent=None, action=None):
        self.state = state
        self.parent = parent
        self.action = action

# ==============================
# QueueFrontier class (for BFS)
# ==============================
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

# ==============================
# Maze class
# ==============================
class Maze:
    def __init__(self, filename):
        with open(filename) as f:
            contents = f.read()

        if contents.count("A") != 1 or contents.count("B") != 1:
            raise Exception("Maze must have exactly one start (A) and one goal (B)")

        self.grid = [list(line) for line in contents.splitlines()]
        self.height = len(self.grid)
        self.width = max(len(row) for row in self.grid)

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

        self.solution = None
        self.explored_nodes = set()

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

# ==============================
# DFS Class (جستجوی عمق‌اول)
# ==============================
class DFS:
    def __init__(self, maze):
        self.maze = maze

    def solve(self):
        start_node = Node(self.maze.start)
        frontier = [start_node]
        came_from = {self.maze.start: None}
        steps = []

        while frontier:
            node = frontier.pop()
            state = node.state

            if state == self.maze.goal:
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            for action, neighbor in self.maze.neighbors(state):
                if neighbor not in came_from:
                    frontier.append(Node(neighbor, node, action))
                    came_from[neighbor] = state
                    steps.append(neighbor)

        return steps

# ==============================
# BFS Class (جستجوی سطح‌به‌سطح)
# ==============================
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
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            for action, neighbor in self.maze.neighbors(state):
                if neighbor not in came_from:
                    frontier.add(Node(neighbor, node, action))
                    came_from[neighbor] = state
                    steps.append(neighbor)

        return steps

# ==============================
# AStar Class (جستجوی A*)
# ==============================
class AStar:
    def __init__(self, maze):
        self.maze = maze

    def solve(self):
        def heuristic(a, b):
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
                self.maze.solution = []
                while state != self.maze.start:
                    self.maze.solution.append(state)
                    state = came_from[state]
                self.maze.solution.reverse()
                return steps

            for action, neighbor in self.maze.neighbors(state):
                new_cost = cost_so_far[state] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, self.maze.goal)
                    frontier.append((priority, Node(neighbor, node, action)))
                    came_from[neighbor] = state
                    steps.append(neighbor)

        return steps

# ==============================
# MazeVisualizer class
# ==============================
class MazeVisualizer:
    def __init__(self, maze, algorithm="bfs"):
        pygame.init()

        self.maze = maze
        self.cell_size = 15
        self.width = self.maze.width * self.cell_size
        self.height = self.maze.height * self.cell_size

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(f"Maze | {algorithm.upper()} Algorithm")

        self.wall_img = pygame.image.load("./blocks/wall.png")
        self.path_img = pygame.image.load("./blocks/path.png")
        self.start_img = pygame.image.load("./blocks/start.png")
        self.goal_img = pygame.image.load("./blocks/goal.png")
        self.visited_img = pygame.image.load("./blocks/visited.png")
        self.solution_img = pygame.image.load("./blocks/solution.png")

        self.wall_img = pygame.transform.scale(self.wall_img, (self.cell_size, self.cell_size))
        self.path_img = pygame.transform.scale(self.path_img, (self.cell_size, self.cell_size))
        self.start_img = pygame.transform.scale(self.start_img, (self.cell_size, self.cell_size))
        self.goal_img = pygame.transform.scale(self.goal_img, (self.cell_size, self.cell_size))
        self.visited_img = pygame.transform.scale(self.visited_img, (self.cell_size, self.cell_size))
        self.solution_img = pygame.transform.scale(self.solution_img, (self.cell_size, self.cell_size))

        self.choose_algorithm(algorithm)
        self.run_visualization()

    def choose_algorithm(self, algorithm_type):
        if algorithm_type.lower() == "dfs":
            self.pathfinding = DFS(self.maze)
        elif algorithm_type.lower() == "bfs":
            self.pathfinding = BFS(self.maze)
        elif algorithm_type.lower() == "a_star":
            self.pathfinding = AStar(self.maze)
        else:
            raise ValueError("Unknown algorithm: " + algorithm_type)

        start_time = time.time()
        self.steps = self.pathfinding.solve()
        end_time = time.time()
        self.total_time = end_time - start_time

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
            self.screen.blit(self.visited_img, (x, y))
            pygame.display.update()
            time.sleep(0.01)
        self.draw_solution_path()

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

    def run_visualization(self):
        running = True
        self.draw_maze()
        pygame.display.update()
        self.explore_maze_step_by_step()

        total_steps = len(self.steps)
        solution_length = len(self.maze.solution) if self.maze.solution else 0
        print("Maze solved!")
        print(f"Total nodes checked: {total_steps}")
        print(f"Solution length (from start to goal): {solution_length}")
        print(f"Total time taken: {self.total_time:.4f} seconds")

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit()

# ==============================
# Run the app
# ==============================
if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit("Usage: python main.py ./mazemaps/maze.txt algorithm\nExample: python main.py ./mazemaps/maze.txt bfs")
    maze = Maze(sys.argv[1])
    algorithm = sys.argv[2]  # "dfs", "bfs" or "a_star"
    MazeVisualizer(maze, algorithm)
