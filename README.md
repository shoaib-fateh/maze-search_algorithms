# Maze Search Algorithms

### Description:
This project compares three popular maze-solving algorithms: **Depth-First Search (DFS)**, **Breadth-First Search (BFS)**, and **A\* (A-star)**. The goal of the project is to explore and analyze how each algorithm performs when solving mazes. The user can visualize the maze-solving process and compare the performance and efficiency of each algorithm.

The project includes:
- Visualization of maze solving steps.
- A simple maze editor to create custom mazes.
- The implementation of DFS, BFS, and A* algorithms for pathfinding.

### Purpose:
The purpose of this project is to help users understand the mechanics and performance differences between DFS, BFS, and A* in solving maze problems. It’s ideal for educational purposes, teaching algorithm efficiency, and helping users visualize algorithm execution step by step.

---

### Features:
- **DFS (Depth-First Search):** Explores as far as possible along each branch before backtracking.
- **BFS (Breadth-First Search):** Explores all neighbors at the present depth before moving on to nodes at the next depth level.
- **A* (A-star):** Uses both the actual distance from the start and a heuristic estimate to find the shortest path efficiently.

---

### Installation:
To run the project, follow these steps:

1. Clone the repository:

   ```bash
   git clone https://github.com/shoaib-fateh/maze-search_algorithms.git
   ```

2. Navigate to the project directory:

   ```bash
   cd maze-search_algorithms
   ```

3. Install the required dependencies:

   ```bash
   pip install -r requirements.txt
   ```

---

### Usage:

1. **Editing Mazes:**
   - With `mazeeditor.py` you can edit the maze manually or load a pre-existing maze map to visualize how the algorithms work on it.

2. **Running the Algorithms:**
   - In the `main.py` file, you can run the three algorithms (DFS, BFS, and A*) on a given maze. Each algorithm’s output will be visualized step-by-step.
   - After the solution is found, the following information will be printed:
     - **Maze solved!**
     - **Total nodes checked**
     - **Solution length (from start to goal)**
     - **Total time taken** for solving the maze

   Example command:
   ```bash
   python main.py ./mazemaps/maze.txt bfs
   ```

3. **Viewing Results:**
   - The results will show the solution path, the nodes explored, and the time taken for each algorithm to solve the maze.

---

### How It Works:

- **DFS Algorithm:** Starts at the beginning of the maze and explores as far as possible along each branch before backtracking.
- **BFS Algorithm:** Explores all neighboring nodes at the present depth level before moving on to the next depth level.
- **A* Algorithm:** Uses a heuristic to prioritize nodes with the shortest estimated cost to reach the goal.

---

### Example Output:
- Maze solved!
- Total nodes checked: 345
- Solution length (from start to goal): 128
- Total time taken: 2.3423 seconds

---

### Contributing:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/your-feature`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

---

### License:
This project is open-source and available under the [MIT License](LICENSE).

---

### Acknowledgments:
- Special thanks to the contributors and community who helped improve this project.
- Any resources or tutorials referenced in the development of this project.