# 🏭 Warehouse Delivery Route Planner (Live A* Search)

An interactive, visual AI pathfinding application simulating an **Automated Guided Vehicle (AGV)** navigating an obstacle-dense warehouse floor using the **A\* (A-Star) Search Algorithm**.

Designed as an educational and demonstration tool for **Artificial Intelligence (AI) / Fundamentals of AI (FAI)** coursework, this application enables teachers and students to inspect, step through, and analyze heuristic search and path optimization in real time.

---

## 📌 1. Project Overview & Problem Statement

### 1.1 Problem Statement
In automated warehousing (such as Amazon Fulfillment Centers or automated logistics hubs), Autonomous Mobile Robots (AMRs) and Automated Guided Vehicles (AGVs) must transport parcels between pick-up and drop-off stations. The environment contains fixed storage racks, narrow aisles, and dynamic obstacles. 

The objective is to compute the **optimal (minimum-cost) collision-free trajectory** from the starting station to the delivery destination while minimizing computational overhead (number of nodes expanded).

### 1.2 Core Objectives
- **Demonstrate Heuristic Search**: Move beyond theoretical equations by visualizing how $f(n) = g(n) + h(n)$ guides the search towards the goal.
- **Provide Step-by-Step Transparency**: Every single node expansion, neighbour evaluation, edge relaxation, and queue update is traced in real time.
- **Interactive Experimentation**: Allow users to alter obstacles, randomize layouts, step forward manually, and observe how heuristic pruning responds to bottlenecks and dead ends.

---

## 🧠 2. Mathematical & Algorithmic Foundations

The application implements the standard **A\* Search Algorithm** on a 2D discrete grid graph $G = (V, E)$.

### 2.1 The Evaluation Function
At each node $n$, A\* evaluates:
$$f(n) = g(n) + h(n)$$

Where:
- **$g(n)$ (Exact Path Cost)**: The cumulative cost incurred to travel from the starting cell $S$ to current cell $n$. In this grid, moving 1 orthogonal step has a uniform cost of $c(u, v) = 1$.
- **$h(n)$ (Heuristic Estimate)**: The estimated cost from node $n$ to the destination cell $D$.
- **$f(n)$ (Total Estimated Cost)**: The estimated total cost of the cheapest path through $n$ from $S$ to $D$.

### 2.2 Heuristic Function: Manhattan Distance
Because the robot can move only in 4 cardinal directions (Up, Down, Left, Right) without diagonal movement, the **Manhattan Distance ($L_1$ norm)** is employed:

$$h(n) = |r_n - r_{goal}| + |c_n - c_{goal}|$$

#### Proof of Optimality Properties:
1. **Admissibility ($h(n) \le h^*(n)$)**:
   In an obstacle-free grid, the shortest distance between $(r_n, c_n)$ and $(r_g, c_g)$ with 4-way movement is exactly the Manhattan distance. The presence of obstacle shelves can only increase or keep equal the true path cost $h^*(n)$. Thus, $h(n)$ never overestimates the true cost ($h(n) \le h^*(n)$), guaranteeing that **A\* finds an optimal (shortest) path**.
2. **Consistency / Monotonicity**:
   For any adjacent nodes $n$ and $n'$, the triangle inequality holds:
   $$h(n) \le c(n, n') + h(n')$$
   Since $c(n, n') = 1$ and $|h(n) - h(n')| \le 1$, consistency holds. This guarantees that once a node is placed into the `CLOSED` set, the shortest path to it has already been discovered, eliminating the need to re-expand nodes.

### 2.3 Neighbour Evaluation Decisions
During node expansion, neighbours are categorized into one of four states:
- **`ADD`**: A previously unvisited node is discovered; its $g$, $h$, and $f$ scores are recorded, and it is pushed onto the `OPEN` priority queue.
- **`UPDATE` (Edge Relaxation)**: A better (lower $g$) path to a node already in `OPEN` is found. Its parent pointer and scores are updated.
- **`CLOSED`**: The neighbour has already been expanded and is in the `CLOSED` set; it is skipped.
- **`KEEP OLD`**: The neighbour is reachable via an alternate path, but the current path is equal to or more costly than the previously known route.

---

## 🏗️ 3. Architecture & Data Structures

The project is built in Python using zero external packages, leveraging built-in standard library structures for high performance and clean memory management:

```
                  +-----------------------------------+
                  |        WarehousePlanner           |
                  +-----------------------------------+
                     |              |              |
                     v              v              v
               [Grid Canvas]   [A* Engine]   [Trace Panel]
                     |              |              |
        - 15x20 Grid Cells          |        - Step Counter
        - Interactive Walls         |        - Current g, h, f
        - Parent Links (Dashed)     |        - Neighbour Log
        - Final Path (Yellow)       |        - OPEN Priority List
                                    |
                    +---------------+---------------+
                    |                               |
                    v                               v
         [Priority Queue / OPEN]             [Sets & Mappings]
         - heapq min-heap                    - open_nodes: set()
         - (f, counter, cell)                - closed_nodes: set()
         - Tie-breaker counter               - walls: set()
                                             - parent: dict()
                                             - g_score: dict()
                                             - f_score: dict()
```

### Key Data Structures:
| Structure | Type | Purpose |
| :--- | :--- | :--- |
| `frontier` | `heapq` (Min-Heap) | Priority queue storing `(f_score, tie_breaker_id, node)` for $O(\log N)$ extraction of minimum-$f$ nodes. |
| `open_nodes` | `set` | $O(1)$ membership checking for active nodes in the frontier. |
| `closed_nodes` | `set` | $O(1)$ tracking of visited/expanded nodes. |
| `parent` | `dict` | Back-pointer tree (`child -> parent`) used for reconstructing the optimal route. |
| `g_score`, `f_score` | `dict` | Exact score lookups per cell coordinates `(row, col)`. |
| `walls` | `set` | Storage rack obstacles preventing robot traversal. |

---

## 🎨 4. Graphical Interface & Visual Legend

The application uses an ergonomic dark-themed interface built on `tkinter`.

### 4.1 Map Legend
| Symbol / Color | Name | Description |
| :---: | :--- | :--- |
| 🟩 `#22c55e` | **Start (S)** | Fixed AGV depot / starting bay `(1, 1)` |
| 🟥 `#ef4444` | **Destination (D)** | Parcel drop-off destination `(13, 18)` |
| ⬛ `#334155` | **Shelf / Obstacle** | Impassable warehouse racks |
| 🟪 `#c084fc` | **Current Node** | Node currently popped from `OPEN` and being expanded |
| 🟧 `#fb923c` | **Neighbours** | Candidate adjacent cells currently being evaluated |
| 🟦 `#93c5fd` | **OPEN List** | Discovered frontier cells waiting to be explored |
| ⬜ `#cbd5e1` | **CLOSED List** | Fully expanded cells |
| 🟨 `#fbbf24` | **Final Route** | Backtracked optimal path from Start to Destination |
| ╌╌ `#64748b` | **Parent Links** | Dotted lines illustrating the expanding search tree |

### 4.2 Side Panel: Real-Time Telemetry
The right-hand panel gives complete diagnostic insight into the algorithm's state:
1. **Current Node Telemetry**: Displays the active coordinate alongside its exact $g$, $h$, and $f$ scores.
2. **Neighbour Evaluation Log**: Tabular breakdown showing tentative $g$, $h$, $f$ and decision (`ADD`, `UPDATE`, `CLOSED`, or `KEEP OLD`).
3. **OPEN List Live Queue**: Shows the priority queue candidates sorted by lowest $f$, showing exactly which cell will be picked next.
4. **Aggregate Search Statistics**: Displays total nodes expanded, active nodes in `OPEN`, and nodes in `CLOSED`.

---

## 🕹️ 5. Controls & User Guide

| Control | Action / Purpose |
| :--- | :--- |
| **RUN LIVE A\*** | Starts continuous real-time execution of the search at the selected speed. |
| **STEP ONCE** | Executes **exactly one** node expansion. Ideal for walking a teacher through the algorithm step-by-step. |
| **PAUSE** | Pauses live execution to inspect node values or discuss search state. |
| **SHOW FINAL PATH** | Backtracks along parent pointers and highlights the final optimal route in yellow. |
| **RANDOM SHELVES** | Generates a new random warehouse obstacle distribution (~16% density). |
| **RESET** | Restores the standard 6-block warehouse shelf layout and clears all search overlays. |
| **Animation delay Slider** | Adjusts speed dynamically between 60 ms (fast) and 900 ms (detailed observation). |
| **Grid Left-Click** | Toggle a shelf/obstacle on any cell (except S and D) when the search is paused. |
| **Escape (Esc)** | Toggle between windowed and fullscreen display. |

---

## ⚙️ 6. System Requirements & Installation

This project is built purely on Python's core libraries. **No `pip install` commands are needed.**

### Prerequisites
- **Python 3.8+** (Python 3.8, 3.9, 3.10, 3.11, 3.12, or newer)
- **Tkinter** (bundled by default with standard Python installations on Windows and macOS)

### Running the Application

1. Open your terminal or Command Prompt in the project folder:
   ```bash
   cd d:/fai
   ```

2. Run the Python script:
   ```bash
   python warehouse.py
   ```

---

## 🎓 7. Teacher / Examiner Demonstration Guide (Viva Voce Prep)

When presenting this project for an evaluation, viva, or course project review, follow this structured walkthrough:

### Step 1: Explain the Real-World Scenario
> *"This project models an Automated Guided Vehicle (AGV) in a smart logistics warehouse. The green cell (S) is the robot bay, the red cell (D) is the delivery point, and dark blocks represent warehouse shelf aisles."*

### Step 2: Demonstrate Single-Step Expansion (`STEP ONCE`)
1. Click **`RESET`**, then click **`STEP ONCE`** 3 to 4 times.
2. Direct the teacher's attention to the **Side Panel**:
   - Point out the active coordinate $(r, c)$ and its $g$, $h$, and $f$ values.
   - Show how the algorithm explores neighbors in 4 directions, calculates tentative $g(n) = g(\text{current}) + 1$, evaluates Manhattan distance $h(n)$, and pushes candidates to `OPEN`.
   - Show the dashed parent links pointing back toward the source node.

### Step 3: Run Live Search & Highlight Heuristic Directness
1. Set the animation delay slider to ~150–250 ms.
2. Click **`RUN LIVE A*`**.
3. Point out how the search is directed toward the destination (bottom-right) rather than expanding uniformly in all directions like Dijkstra or BFS. This proves the heuristic $h(n)$ is successfully guiding the search.

### Step 4: Show Final Path Reconstruction
1. Once the destination is selected, click **`SHOW FINAL PATH`**.
2. Explain how the optimal trajectory (yellow path) is reconstructed in $O(L)$ time by starting at the goal and following parent references back to the start node.

### Step 5: Test Obstacle Avoidance & Dead Ends
1. Click on the grid to create a wall closing an aisle or trapping a pathway.
2. Click **`RUN LIVE A*`** to show how A\* detects the dead end, leaves it in `CLOSED`, and backtracks to expand alternative candidate paths in the `OPEN` queue.


- **Time Complexity**: $O(b^d)$ in the worst case (where $b$ is the branching factor $\le 4$, and $d$ is the path depth). With a consistent heuristic, each state is expanded at most once, bounding time by $O(|V| \log |V|)$ on this grid.
- **Space Complexity**: $O(|V|)$ because all visited and frontier nodes are maintained in memory in the `OPEN` list and `CLOSED` set.
---

## 📁 9. Repository Structure

```
d:/fai/
│
├── warehouse.py     # Main application file containing the GUI and A* engine
└── README.md        # Comprehensive technical documentation & presentation guide
```