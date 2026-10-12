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

OPEN_COLOR = "#93c5fd"       # discovered nodes waiting in OPEN

CLOSED_COLOR = "#cbd5e1"     # expanded nodes

CURRENT_COLOR = "#c084fc"    # node currently being expanded

NEIGHBOR_COLOR = "#fb923c"   # neighbours being evaluated

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





class WarehousePlanner:

    """Live, incremental A*: each animation tick performs one real node expansion."""

    def __init__(self, root):

        self.root = root

        self.root.title("Warehouse Delivery Route Planner — Live A* Search")

        self.root.configure(bg=BG)

        try:

            self.root.state("zoomed")

        except tk.TclError:

            self.root.attributes("-fullscreen", True)

        self.root.bind("<Escape>", self.toggle_fullscreen)

        self.fullscreen = False



        screen_w = root.winfo_screenwidth()

        screen_h = root.winfo_screenheight()

        self.cell_size = max(24, min(42, (screen_w - 520) // COLS, (screen_h - 260) // ROWS))

        self.grid_width = COLS * self.cell_size

        self.grid_height = ROWS * self.cell_size



        self.start = START

        self.goal = DESTINATION

        self.walls = set()

        self.path = []

        self.open_nodes = set()

        self.closed_nodes = set()

        self.current_node = None

        self.neighbor_nodes = set()

        self.robot_position = self.start

        self.parent = {}

        self.g_score = {}

        self.f_score = {}

        self.frontier = []

        self.counter = count()

        self.search_running = False

        self.animation_id = None

        self.started_at = 0

        self.expanded_count = 0

        self.step_number = 0

        self.speed_ms = tk.IntVar(value=350)

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

                 font=("Segoe UI", 22, "bold"), bg=BG, fg=TEXT).pack(pady=(8, 1))

        tk.Label(self.root, text="LIVE A* SEARCH  •  NEIGHBOUR EVALUATION  •  BEST-CANDIDATE SELECTION",

                 font=("Segoe UI", 10, "bold"), bg=BG, fg="#38bdf8").pack(pady=(0, 8))



        body = tk.Frame(self.root, bg=BG)

        body.pack(fill="both", expand=True, padx=12, pady=4)



        left = tk.Frame(body, bg=BG)

        left.pack(side="left", anchor="n")

        legend = tk.Frame(left, bg=BG)

        legend.pack(pady=(0, 5))

        legend_items = [

            (START_COLOR, "Start"), (GOAL_COLOR, "Destination"), (WALL, "Shelf"),

            (OPEN_COLOR, "OPEN"), (CLOSED_COLOR, "CLOSED"),

            (CURRENT_COLOR, "Current"), (NEIGHBOR_COLOR, "Neighbours"), (PATH_COLOR, "Final path")

        ]

        for i, (color, label) in enumerate(legend_items):

            item = tk.Frame(legend, bg=BG)

            item.grid(row=i // 4, column=i % 4, padx=5, sticky="w")

            tk.Label(item, text="■", font=("Segoe UI", 12), bg=BG, fg=color).pack(side="left")

            tk.Label(item, text=label, font=("Segoe UI", 9), bg=BG, fg=TEXT).pack(side="left", padx=(2, 0))



        self.canvas = tk.Canvas(left, width=self.grid_width, height=self.grid_height,

                                bg=FLOOR, highlightthickness=0)

        self.canvas.pack()

        self.canvas.bind("<Button-1>", self.on_grid_click)



        right = tk.Frame(body, bg=PANEL, padx=14, pady=12, width=390)

        right.pack(side="left", fill="y", padx=(12, 0))

        right.pack_propagate(False)

        tk.Label(right, text="A* SEARCH TRACE", font=("Segoe UI", 15, "bold"),

                 bg=PANEL, fg=TEXT).pack(anchor="w")

        tk.Label(right, text="Each tick expands one real node from OPEN.",

                 font=("Segoe UI", 9), bg=PANEL, fg=MUTED, wraplength=350).pack(anchor="w", pady=(2, 10))



        self.step_label = tk.Label(right, text="Step 0 — Ready", font=("Segoe UI", 12, "bold"),

                                   bg=PANEL, fg="#38bdf8")

        self.step_label.pack(anchor="w", pady=(0, 5))

        self.current_info = tk.Label(right, text="Current node: —\ng = —   h = —   f = —",

                                     font=("Consolas", 11), bg=PANEL, fg=TEXT, justify="left")

        self.current_info.pack(anchor="w", pady=(0, 8))

        self.action_text = tk.Label(right, text="Click RUN LIVE A* to watch the search.",

                                    font=("Segoe UI", 10), bg=PANEL, fg=TEXT,

                                    justify="left", wraplength=350, anchor="w")

        self.action_text.pack(fill="x", anchor="w", pady=(0, 8))



        tk.Label(right, text="NEIGHBOUR EVALUATION", font=("Segoe UI", 10, "bold"),

                 bg=PANEL, fg="#fb923c").pack(anchor="w", pady=(3, 3))

        self.neighbor_text = tk.Text(right, height=7, width=43, font=("Consolas", 9),

                                     bg="#0b1220", fg=TEXT, relief="flat", wrap="none")

        self.neighbor_text.pack(fill="x")

        self.neighbor_text.insert("1.0", "Neighbour details appear here.\n")

        self.neighbor_text.configure(state="disabled")



        tk.Label(right, text="OPEN LIST — lowest f selected next", font=("Segoe UI", 10, "bold"),

                 bg=PANEL, fg="#93c5fd").pack(anchor="w", pady=(10, 3))

        self.open_text = tk.Text(right, height=5, width=43, font=("Consolas", 9),

                                 bg="#0b1220", fg=TEXT, relief="flat")

        self.open_text.pack(fill="x")

        self.open_text.insert("1.0", "OPEN: not initialized\n")

        self.open_text.configure(state="disabled")



        self.stats = tk.Label(right, text="Expanded: 0   |   OPEN: 0   |   CLOSED: 0",

                              font=("Segoe UI", 10, "bold"), bg=PANEL, fg=TEXT, justify="left")

        self.stats.pack(anchor="w", pady=(9, 6))



        controls = tk.Frame(self.root, bg=BG)

        controls.pack(pady=7)

        self.make_button(controls, "RUN LIVE A*", self.start_search, "#22c55e", BG)

        self.make_button(controls, "STEP ONCE", self.step_once, "#38bdf8", BG)

        self.make_button(controls, "PAUSE", self.pause_search, "#fbbf24", BG)

        self.make_button(controls, "SHOW FINAL PATH", self.show_final_path, "#a78bfa", BG)

        self.make_button(controls, "RANDOM SHELVES", self.random_obstacles, "#f59e0b", BG)

        self.make_button(controls, "RESET", self.reset, "#475569", TEXT)



        speed_row = tk.Frame(self.root, bg=BG)

        speed_row.pack(pady=(0, 3))

        tk.Label(speed_row, text="Animation delay:", font=("Segoe UI", 9), bg=BG, fg=MUTED).pack(side="left")

        tk.Scale(speed_row, from_=60, to=900, orient="horizontal", variable=self.speed_ms,

                 length=180, bg=BG, fg=TEXT, highlightthickness=0, troughcolor=PANEL).pack(side="left")

        tk.Label(self.root, text="Click a cell to toggle a shelf when search is paused. Press Esc to toggle fullscreen.",

                 font=("Segoe UI", 9), bg=BG, fg=MUTED).pack(pady=(0, 6))



    @staticmethod

    def make_button(parent, text, command, bg, fg):

        tk.Button(parent, text=text, command=command, font=("Segoe UI", 9, "bold"),

                  bg=bg, fg=fg, activebackground=bg, activeforeground=fg,

                  relief="flat", bd=0, padx=10, pady=9, cursor="hand2").pack(side="left", padx=3)



    def draw_grid(self):
        self.canvas.delete("all")
        size = self.cell_size
        path_set = set(self.path)

        for r in range(ROWS):
            for c in range(COLS):
                cell = (r, c)
                x1, y1 = c * size, r * size
                x2, y2 = x1 + size, y1 + size

                color = FLOOR
                if cell in self.closed_nodes:
                    color = CLOSED_COLOR
                if cell in self.open_nodes:
                    color = OPEN_COLOR
                if cell in self.neighbor_nodes:
                    color = NEIGHBOR_COLOR
                if cell == self.current_node:
                    color = CURRENT_COLOR
                if cell in path_set:
                    color = PATH_COLOR
                if cell in self.walls:
                    color = WALL
                if cell == self.start:
                    color = START_COLOR
                if cell == self.goal:
                    color = GOAL_COLOR

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline=GRID_LINE,
                    width=1
                )

        # Draw parent links; keep all g, h, and f values in the side panel only.
        for node, parent in self.parent.items():
            if parent is not None and node not in self.walls:
                x1 = parent[1] * size + size / 2
                y1 = parent[0] * size + size / 2
                x2 = node[1] * size + size / 2
                y2 = node[0] * size + size / 2

                self.canvas.create_line(
                    x1, y1, x2, y2,
                    fill="#64748b",
                    width=1,
                    dash=(2, 3)
                )

        # Redraw the start and destination labels above the parent links.
        for cell, label, color in [
            (self.start, "S", "#052e16"),
            (self.goal, "D", "white")
        ]:
            r, c = cell
            self.canvas.create_text(
                c * size + size / 2,
                r * size + size / 2,
                text=label,
                font=("Segoe UI", max(9, size // 3), "bold"),
                fill=color
            )

    def set_text(self, widget, content):

        widget.configure(state="normal")

        widget.delete("1.0", "end")

        widget.insert("1.0", content)

        widget.configure(state="disabled")



    def initialize_search(self):

        self.pause_search()

        self.path = []

        self.open_nodes = {self.start}

        self.closed_nodes = set()

        self.current_node = None

        self.neighbor_nodes = set()

        self.robot_position = self.start

        self.parent = {self.start: None}

        self.g_score = {self.start: 0}

        self.f_score = {self.start: manhattan_distance(self.start, self.goal)}

        self.frontier = [(self.f_score[self.start], next(self.counter), self.start)]

        self.expanded_count = 0

        self.step_number = 0

        self.started_at = time.perf_counter()

        self.step_label.config(text="Step 0 — Initialized")

        self.current_info.config(text=f"Current node: {self.start}\ng = 0   h = {manhattan_distance(self.start, self.goal)}   f = {manhattan_distance(self.start, self.goal)}")

        self.action_text.config(text="OPEN contains only the start node. Select the lowest-f node, then inspect its neighbours.")

        self.set_text(self.neighbor_text, "Start node added to OPEN.\n")

        self.refresh_open_list()

        self.refresh_stats()

        self.draw_grid()



    def start_search(self):

        if self.search_running:

            return

        self.initialize_search()

        self.search_running = True

        self.auto_tick()



    def auto_tick(self):

        if not self.search_running:

            return

        if self.perform_one_expansion():

            self.animation_id = self.root.after(self.speed_ms.get(), self.auto_tick)



    def step_once(self):

        if self.search_running:

            self.pause_search()

        if not self.g_score:

            self.initialize_search()

        self.perform_one_expansion()



    def perform_one_expansion(self):

        while self.frontier:

            f, _, current = heapq.heappop(self.frontier)

            if current in self.closed_nodes:

                continue

            if f != self.f_score.get(current):

                continue

            break

        else:

            self.search_running = False

            self.current_node = None

            self.step_label.config(text="Search finished — no route")

            self.action_text.config(text="OPEN is empty. No path to the destination exists; edit the shelves and reset.")

            self.draw_grid()

            return False



        self.current_node = current

        self.open_nodes.discard(current)

        self.step_number += 1

        self.robot_position = current

        g = self.g_score[current]

        h = manhattan_distance(current, self.goal)

        f_current = g + h



        if current == self.goal:

            self.closed_nodes.add(current)

            self.search_running = False

            self.step_label.config(text=f"Step {self.step_number} — Destination selected!")

            self.current_info.config(text=f"Current node: {current} (DESTINATION)\ng = {g}   h = {h}   f = {f_current}")

            self.action_text.config(text="The destination is now the lowest-f node removed from OPEN. A* stops and reconstructs the route using parent links.")

            self.set_text(self.neighbor_text, "Goal reached — no further neighbours are expanded.\n")

            self.refresh_open_list()

            self.refresh_stats()

            self.draw_grid()

            return False



        self.closed_nodes.add(current)

        self.expanded_count += 1

        neighbours = get_neighbors(current, self.walls)

        self.neighbor_nodes = set(neighbours)

        lines = [f"Expand {current}: g={g}, h={h}, f={f_current}", "Neighbour | g  h  f | decision"]

        for neighbor in neighbours:

            tentative_g = g + 1

            nh = manhattan_distance(neighbor, self.goal)

            nf = tentative_g + nh

            if neighbor in self.closed_nodes:

                decision = "CLOSED"

            elif tentative_g < self.g_score.get(neighbor, float("inf")):

                old = self.g_score.get(neighbor)

                self.g_score[neighbor] = tentative_g

                self.f_score[neighbor] = nf

                self.parent[neighbor] = current

                heapq.heappush(self.frontier, (nf, next(self.counter), neighbor))

                self.open_nodes.add(neighbor)

                decision = "ADD" if old is None else "UPDATE"

            else:

                decision = "KEEP OLD"

            lines.append(f"{str(neighbor):9} | {tentative_g:2} {nh:2} {nf:2} | {decision}")

        self.step_label.config(text=f"Step {self.step_number} — Expand {current}")

        self.current_info.config(text=f"Current node: {current}\ng = {g}   h = {h}   f = {f_current}")

        self.action_text.config(text="Orange = neighbours being evaluated. Blue = discovered candidates in OPEN. Grey = expanded nodes in CLOSED. Next, A* removes the lowest-f node from OPEN.")

        self.set_text(self.neighbor_text, "\n".join(lines))

        self.refresh_open_list()

        self.refresh_stats()

        self.draw_grid()

        return True



    def refresh_open_list(self):

        valid = []

        for f, _, node in self.frontier:

            if node in self.open_nodes and f == self.f_score.get(node):

                valid.append((f, self.g_score[node], manhattan_distance(node, self.goal), node))

        valid.sort()

        lines = [f"{'Node':9} {'g':>3} {'h':>3} {'f':>3}", "-" * 23]

        for f, g, h, node in valid[:18]:

            lines.append(f"{str(node):9} {g:3} {h:3} {f:3}")

        if len(valid) > 18:

            lines.append(f"... and {len(valid)-18} more")

        if not valid:

            lines.append("(empty)")

        self.set_text(self.open_text, "\n".join(lines))



    def refresh_stats(self):

        self.stats.config(text=f"Expanded: {self.expanded_count}   |   OPEN: {len(self.open_nodes)}   |   CLOSED: {len(self.closed_nodes)}")



    def reconstruct_path(self):

        if self.goal not in self.parent:

            return []

        path = []

        node = self.goal

        while node is not None:

            path.append(node)

            node = self.parent.get(node)

        return list(reversed(path))



    def show_final_path(self):

        if self.search_running:

            self.pause_search()

        if self.goal not in self.parent:

            messagebox.showinfo("Search not finished", "Run the live search until the destination is selected first.")

            return

        self.path = self.reconstruct_path()

        self.neighbor_nodes = set()

        self.current_node = None

        self.robot_position = self.goal

        self.action_text.config(text=f"Shortest path reconstructed from parent links: {len(self.path)-1} moves. Yellow cells show the final route.")

        self.step_label.config(text="Final shortest path")

        self.draw_grid()



    def pause_search(self):

        self.search_running = False

        if self.animation_id is not None:

            try:

                self.root.after_cancel(self.animation_id)

            except tk.TclError:

                pass

            self.animation_id = None



    def on_grid_click(self, event):

        if self.search_running:

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

        self.clear_search_state()

        self.action_text.config(text="Warehouse changed. Run live A* to start a fresh search.")

        self.draw_grid()



    def clear_search_state(self):

        self.pause_search()

        self.path = []

        self.open_nodes = set()

        self.closed_nodes = set()

        self.current_node = None

        self.neighbor_nodes = set()

        self.robot_position = self.start

        self.parent = {}

        self.g_score = {}

        self.f_score = {}

        self.frontier = []

        self.counter = count()

        self.expanded_count = 0

        self.step_number = 0

        self.set_text(self.neighbor_text, "Neighbour details appear here.\n")

        self.set_text(self.open_text, "OPEN: not initialized\n")

        self.current_info.config(text="Current node: —\ng = —   h = —   f = —")

        self.step_label.config(text="Step 0 — Ready")

        self.refresh_stats()



    def random_obstacles(self):

        self.walls.clear()

        for r in range(ROWS):

            for c in range(COLS):

                cell = (r, c)

                if cell not in (self.start, self.goal) and random.random() < 0.16:

                    self.walls.add(cell)

        self.clear_search_state()

        self.action_text.config(text="New random warehouse generated. Run live A* to watch the search.")

        self.draw_grid()



    def reset(self):

        self.walls.clear()

        self.add_sample_shelves()

        self.clear_search_state()

        self.action_text.config(text="Warehouse reset. Click RUN LIVE A* or STEP ONCE.")

        self.draw_grid()





if __name__ == "__main__":

    root = tk.Tk()

    app = WarehousePlanner(root)

    root.mainloop()
