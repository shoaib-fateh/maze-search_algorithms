import tkinter as tk
from tkinter import messagebox
import sys

class MazeEditor:
    def __init__(self, root, maze_file):
        self.root = root
        self.root.title("Maze Editor")

        # Load the maze and determine rows and cols automatically
        self.maze, self.rows, self.cols = self.load_maze(maze_file)
        self.cell_size = 15

        self.selected_block = 'P'  # Default selected block is Path

        self.create_widgets()
        self.draw_maze()

    def load_maze(self, maze_file):
        """Load the maze from a file and determine the size."""
        try:
            with open(maze_file, 'r') as f:
                lines = f.readlines()

            maze = [list(line.strip()) for line in lines]
            
            # Automatically set rows and cols based on the loaded maze
            rows = len(maze)
            cols = max(len(row) for row in maze)  # Find the longest row to set columns

            # Pad all rows to match the longest row length
            for row in maze:
                if len(row) < cols:
                    row.extend([' '] * (cols - len(row)))  # Pad the row with spaces

            return maze, rows, cols

        except FileNotFoundError:
            messagebox.showerror("Error", f"File {maze_file} not found!")
            self.root.quit()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load maze: {e}")
            self.root.quit()

    def create_widgets(self):
        self.canvas = tk.Canvas(self.root, width=self.cols * self.cell_size, height=self.rows * self.cell_size)
        self.canvas.pack()

        self.create_buttons()

    def create_buttons(self):
        self.path_button = tk.Button(self.root, text="Path", command=lambda: self.select_block('P'))
        self.path_button.pack(side=tk.LEFT)

        self.wall_button = tk.Button(self.root, text="Wall", command=lambda: self.select_block('#'))
        self.wall_button.pack(side=tk.LEFT)

        self.start_button = tk.Button(self.root, text="Start", command=lambda: self.select_block('A'))
        self.start_button.pack(side=tk.LEFT)

        self.goal_button = tk.Button(self.root, text="Goal", command=lambda: self.select_block('B'))
        self.goal_button.pack(side=tk.LEFT)

        self.erase_button = tk.Button(self.root, text="Erase", command=self.erase_block)
        self.erase_button.pack(side=tk.LEFT)

        self.save_button = tk.Button(self.root, text="Save", command=self.save_maze)
        self.save_button.pack(side=tk.LEFT)

    def select_block(self, block_type):
        """Select a block type and update the selected block."""
        self.selected_block = block_type
        print(f"Selected: {block_type}")

    def erase_block(self):
        """Set the selected block to empty (erase mode)."""
        self.selected_block = ' '  # Empty block
        print("Erase Mode")

    def draw_maze(self):
        """Draw the maze on the canvas."""
        self.canvas.delete("all")  # Clear the canvas
        for i in range(self.rows):
            for j in range(self.cols):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                cell = self.maze[i][j]

                color = 'white'  # Default path color
                if cell == '#':
                    color = 'black'  # Wall color
                elif cell == 'A':
                    color = 'green'  # Start color
                elif cell == 'B':
                    color = 'red'  # Goal color

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='gray', tags=f"{i},{j}")

                # Bind click event for each cell to change its state
                self.canvas.tag_bind(f"{i},{j}", "<Button-1>", lambda event, row=i, col=j: self.update_cell(row, col))

    def update_cell(self, row, col):
        """Update the selected cell with the current selected block."""
        self.maze[row][col] = self.selected_block
        self.draw_maze()  # Redraw the maze with the updated cell

    def save_maze(self):
        """Save the current maze to the file specified in cmd."""
        maze_file = sys.argv[1] if len(sys.argv) > 1 else 'maze.txt'  # Get file name from cmd or default
        try:
            with open(maze_file, 'w') as f:
                for row in self.maze:
                    f.write("".join(row) + '\n')
            messagebox.showinfo("Save", f"Maze saved to {maze_file} successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save maze: {e}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python mazeeditor.py <maze_file>")
        sys.exit(1)

    maze_file = sys.argv[1]  # Get the maze file name from the command line
    root = tk.Tk()
    maze_editor = MazeEditor(root, maze_file=maze_file)
    root.mainloop()

if __name__ == "__main__":
    main()
