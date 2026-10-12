import tkinter as tk
from tkinter import messagebox
import heapq
import random
import time
from itertools import count

ROWS, COLS = 15, 20
START = (1, 1)
DESTINATION = (13, 18)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

BG = "#101827"
PANEL = "#192538"
FLOOR = "#e8eef5"
GRID_LINE = "#cbd5e1"
WALL = "#334155"
START_COLOR = "#22c55e"
GOAL_COLOR = "#ef4444"
PATH_COLOR = "#fbbf24"
EXPLORED_COLOR = "#bae6fd"
ROBOT_COLOR = "#0284c7"
TEXT = "#f1f5f9"
MUTED = "#9aabc0"


def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(cell, walls):
    row, col = cell
    result = []
    for dr, dc in DIRECTIONS:
        nr, nc = row + dr, col + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and (nr, nc) not in walls:
            result.append((nr, nc))
    return result


def astar(start, goal, walls):
    tie_breaker = count()
    frontier = [(manhattan_distance(start, goal), 0, next(tie_breaker), start)]
    best_cost = {start: 0}
    parent = {start: None}
    expanded = 0
    explored = []

    while frontier:
        _, cost, _, current = heapq.heappop(frontier)
        if cost != best_cost.get(current):
            continue
        if current == goal:
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path)), expanded, explored
        expanded += 1
        explored.append(current)
        for neighbor in get_neighbors(current, walls):
            new_cost = cost + 1
            if new_cost < best_cost.get(neighbor, float("inf")):
                best_cost[neighbor] = new_cost
                parent[neighbor] = current
                priority = new_cost + manhattan_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, new_cost, next(tie_breaker), neighbor))
    return None, expanded, explored


def reachable_cells(start, walls):
    """Return every reachable free grid cell from start (search space, not every route sequence)."""
    queue = [start]
    seen = {start}
    order = []
    while queue:
        current = queue.pop(0)
        order.append(current)
        for neighbor in get_neighbors(current, walls):
            if neighbor not in seen:
                seen.add(neighbor)
                queue.append(neighbor)
    return order


class WarehousePlanner:
    def __init__(self, root):
        self.root = root
        self.root.title("A* Warehouse Delivery Route Planner")
        self.root.configure(bg=BG)
        self.root.state("zoomed")  # Maximize on Windows; uses the full available screen.
        self.root.bind("<Escape>", self.toggle_fullscreen)
        self.fullscreen = False

        # Scale grid cells to the current screen while keeping room for controls.
        screen_w = root.winfo_screenwidth()
        screen_h = root.winfo_screenheight()
        self.cell_size = max(25, min(48, (screen_w - 100) // COLS, (screen_h - 300) // ROWS))
        self.grid_width = COLS * self.cell_size
        self.grid_height = ROWS * self.cell_size

        self.start = START
        self.goal = DESTINATION
        self.walls = set()
        self.path = []
        self.animation_path = []
        self.search_cells = []
        self.explored = []
        self.robot_position = self.start
        self.animation_id = None
        self.animating = False
        self.last_search_time = 0.0
        self.last_expanded = 0
        self.mode = "idle"
        self.add_sample_shelves()
        self.build_ui()
        self.draw_grid()

    def toggle_fullscreen(self, event=None):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def add_sample_shelves(self):
        blocks = [
            (range(2, 6), range(4, 7)),
            (range(8, 12), range(4, 7)),
            (range(2, 6), range(10, 13)),
            (range(8, 12), range(10, 13)),
            (range(2, 6), range(15, 18)),
            (range(8, 12), range(15, 18)),
        ]
        for row_range, col_range in blocks:
            for r in row_range:
                for c in col_range:
                    cell = (r, c)
                    if cell not in (self.start, self.goal):
                        self.walls.add(cell)

    def build_ui(self):
        tk.Label(self.root, text="WAREHOUSE DELIVERY ROUTE PLANNER",
                 font=("Segoe UI", 24, "bold"), bg=BG, fg=TEXT).pack(pady=(12, 2))
        tk.Label(self.root, text="A* SEARCH  •  ROBOT NAVIGATION  •  OBSTACLE AVOIDANCE",
                 font=("Segoe UI", 11, "bold"), bg=BG, fg="#38bdf8").pack(pady=(0, 8))

        legend = tk.Frame(self.root, bg=BG)
        legend.pack(pady=(0, 6))
        for color, label in [
            (START_COLOR, "Robot start"), (GOAL_COLOR, "Delivery point"),
            (WALL, "Shelves"), (PATH_COLOR, "Shortest route"),
            (EXPLORED_COLOR, "Reachable / explored cells")
        ]:
            item = tk.Frame(legend, bg=BG)
            item.pack(side="left", padx=9)
            tk.Label(item, text="■", font=("Segoe UI", 13), bg=BG, fg=color).pack(side="left")
            tk.Label(item, text=label, font=("Segoe UI", 10), bg=BG, fg=TEXT).pack(side="left", padx=(3, 0))

        self.canvas = tk.Canvas(self.root, width=self.grid_width, height=self.grid_height,
                                bg=FLOOR, highlightthickness=0)
        self.canvas.pack(padx=10, pady=5)
        self.canvas.bind("<Button-1>", self.on_grid_click)

        controls = tk.Frame(self.root, bg=BG)
        controls.pack(pady=8)
        self.make_button(controls, "FIND BEST ROUTE", self.find_route, "#22c55e", BG)
        self.make_button(controls, "RUN ROBOT / SHOW PATHS", self.run_robot, "#38bdf8", BG)
        self.make_button(controls, "RANDOM OBSTACLES", self.random_obstacles, "#fbbf24", BG)
        self.make_button(controls, "CLEAR ROUTE", self.clear_route, "#334155", TEXT)
        self.make_button(controls, "RESET", self.reset, "#475569", TEXT)

        tk.Label(self.root, text="Click a grid cell (except S or D) to add/remove a shelf. Press Esc to toggle fullscreen.",
                 font=("Segoe UI", 10), bg=BG, fg=MUTED).pack(pady=(0, 5))
        self.stats = tk.Label(self.root, text="Choose FIND BEST ROUTE or RUN ROBOT / SHOW PATHS.",
                              font=("Segoe UI", 11), bg=PANEL, fg=TEXT, padx=10, pady=9, justify="center")
        self.stats.pack(fill="x", padx=25, pady=(0, 6))
        tk.Label(self.root, text="A* formula: f(n) = g(n) + h(n)   |   h(n) = Manhattan Distance",
                 font=("Consolas", 10), bg=BG, fg="#38bdf8").pack(pady=(0, 7))

    @staticmethod
    def make_button(parent, text, command, bg, fg):
        tk.Button(parent, text=text, command=command, font=("Segoe UI", 10, "bold"),
                  bg=bg, fg=fg, activebackground=bg, activeforeground=fg,
                  relief="flat", bd=0, padx=12, pady=10, cursor="hand2").pack(side="left", padx=4)

    def draw_grid(self):
        self.canvas.delete("all")
        path_set, explored_set = set(self.path), set(self.explored)
        size = self.cell_size
        for r in range(ROWS):
            for c in range(COLS):
                cell = (r, c)
                x1, y1 = c * size, r * size
                x2, y2 = x1 + size, y1 + size
                color = FLOOR
                if cell in explored_set:
                    color = EXPLORED_COLOR
                if cell in path_set:
                    color = PATH_COLOR
                if cell in self.walls:
                    color = WALL
                if cell == self.start:
                    color = START_COLOR
                if cell == self.goal:
                    color = GOAL_COLOR
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=GRID_LINE, width=1)
                if cell == self.start:
                    self.canvas.create_text((x1+x2)/2, (y1+y2)/2, text="S",
                                            font=("Segoe UI", max(10, size//3), "bold"), fill="#052e16")
                elif cell == self.goal:
                    self.canvas.create_text((x1+x2)/2, (y1+y2)/2, text="D",
                                            font=("Segoe UI", max(10, size//3), "bold"), fill="white")
        rr, rc = self.robot_position
        cx, cy = rc * size + size / 2, rr * size + size / 2
        radius = max(7, size // 4)
        self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius,
                                fill=ROBOT_COLOR, outline="white", width=2)

    def on_grid_click(self, event):
        if self.animating:
            return
        col, row = event.x // self.cell_size, event.y // self.cell_size
        if not (0 <= row < ROWS and 0 <= col < COLS):
            return
        cell = (row, col)
        if cell in (self.start, self.goal):
            return
        if cell in self.walls:
            self.walls.remove(cell)
        else:
            self.walls.add(cell)
        self.path, self.explored = [], []
        self.animation_path = []
        self.search_cells = []
        self.robot_position = self.start
        self.mode = "idle"
        self.stats.config(text="Warehouse changed. Choose an action to search again.")
        self.draw_grid()

    def calculate_route(self):
        started = time.perf_counter()
        path, expanded, explored = astar(self.start, self.goal, self.walls)
        elapsed = time.perf_counter() - started
        self.last_search_time = elapsed
        self.last_expanded = expanded
        return path, expanded, explored

    def find_route(self):
        if self.animating:
            return
        path, expanded, explored = self.calculate_route()
        self.path = path or []
        self.explored = []  # Only display the shortest route in this mode.
        self.robot_position = self.start
        self.mode = "best"
        if path is None:
            self.stats.config(text=f"No route found | Expanded nodes: {expanded} | Search time: {self.last_search_time:.5f} s")
            messagebox.showwarning("No route found", "Shelves block all available routes. Remove some shelves and try again.")
        else:
            self.stats.config(text=f"Shortest route: {len(path)-1} steps | Expanded nodes: {expanded} | Search time: {self.last_search_time:.5f} seconds")
        self.draw_grid()

    def run_robot(self):
        """Animate the search area, then move the robot without a yellow route overlay.

        This visualizes explored/reachable cells. It does not enumerate every possible
        route sequence, which can be exponentially large on a grid.
        """
        if self.animating:
            return

        path, expanded, explored = self.calculate_route()
        self.path = []                 # Never draw the yellow shortest-path overlay here.
        self.animation_path = path or []
        self.explored = []
        self.robot_position = self.start
        self.mode = "all"
        self.search_cells = explored

        if path is None:
            self.animation_path = []
            self.stats.config(text=f"No route found | Explored cells: {len(explored)} | Search time: {self.last_search_time:.5f} seconds")
            self.draw_grid()
            messagebox.showwarning("No route found", "Shelves block all available routes. Remove some shelves and try again.")
            return

        self.stats.config(text=f"Exploring warehouse... 0 / {len(explored)} cells | Shortest-route overlay hidden")
        self.draw_grid()
        self.animating = True
        self.animate_exploration(0)

    def animate_exploration(self, index):
        """Reveal A* explored cells one by one, without drawing the best path."""
        if index >= len(self.search_cells):
            self.explored = reachable_cells(self.start, self.walls)
            self.robot_position = self.start
            self.stats.config(text=f"Search complete: {len(self.explored)} reachable cells | Now moving robot; shortest-route overlay hidden")
            self.draw_grid()
            self.animate_step(0)
            return

        self.explored.append(self.search_cells[index])
        self.stats.config(text=f"Exploring warehouse... {index + 1} / {len(self.search_cells)} cells | Shortest-route overlay hidden")
        self.draw_grid()
        self.animation_id = self.root.after(18, lambda: self.animate_exploration(index + 1))

    def animate_step(self, index):
        if index >= len(self.animation_path):
            self.animating = False
            self.animation_id = None
            self.robot_position = self.goal
            self.draw_grid()
            self.stats.config(text=f"Delivery reached! {len(self.explored)} reachable cells shown | Robot travelled {len(self.animation_path)-1} steps. Click FIND BEST ROUTE to show the shortest route in yellow.")
            return
        self.robot_position = self.animation_path[index]
        self.draw_grid()
        self.animation_id = self.root.after(90, lambda: self.animate_step(index + 1))

    def stop_animation(self):
        if self.animation_id is not None:
            self.root.after_cancel(self.animation_id)
            self.animation_id = None
        self.animating = False

    def clear_route(self):
        if self.animating:
            self.stop_animation()
        self.path, self.explored = [], []
        self.animation_path = []
        self.search_cells = []
        self.robot_position = self.start
        self.mode = "idle"
        self.stats.config(text="Route cleared. Find a new route or edit the warehouse.")
        self.draw_grid()

    def random_obstacles(self):
        if self.animating:
            return
        self.walls.clear()
        for r in range(ROWS):
            for c in range(COLS):
                cell = (r, c)
                if cell not in (self.start, self.goal) and random.random() < 0.16:
                    self.walls.add(cell)
        self.path, self.explored = [], []
        self.animation_path = []
        self.search_cells = []
        self.robot_position = self.start
        self.mode = "idle"
        self.stats.config(text="New random warehouse generated. Choose an action to search.")
        self.draw_grid()

    def reset(self):
        if self.animating:
            self.stop_animation()
        self.walls.clear()
        self.add_sample_shelves()
        self.path, self.explored = [], []
        self.animation_path = []
        self.search_cells = []
        self.robot_position = self.start
        self.mode = "idle"
        self.stats.config(text="Warehouse reset. Choose an action to begin.")
        self.draw_grid()


if __name__ == "__main__":
    root = tk.Tk()
    app = WarehousePlanner(root)
    root.mainloop()
