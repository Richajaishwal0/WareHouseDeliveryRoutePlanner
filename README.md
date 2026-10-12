# A\* Warehouse Delivery Route Planner

An interactive Python desktop application simulating an automated delivery robot navigating an obstacle-filled warehouse using the **A\* search algorithm**.

Built with Python and Tkinter, it provides a visual demonstration of pathfinding, heuristic evaluation, and obstacle avoidance in real time.

---

## 🚀 Features

- **A\* Pathfinding Algorithm**: Computes the optimal shortest path using Manhattan Distance heuristic ($f(n) = g(n) + h(n)$).
- **Interactive Grid**: Click any grid cell to dynamically add or remove storage shelves/obstacles.
- **Visual Exploration & Animation**: Watch the algorithm explore candidate cells and see the robot navigate to its destination.
- **Performance Metrics**: Real-time display of search duration, number of expanded nodes, and path step count.
- **Preset & Random Layouts**: Generate random obstacle arrangements or reset back to default warehouse layouts.
- **Zero Third-Party Dependencies**: Built entirely using Python standard libraries (`tkinter`, `heapq`, etc.).

---

## 📋 Requirements

- **Python 3.7+**
- **Tkinter** (included with standard Python installations on Windows and macOS)

---

## 🏃 Getting Started

1. Clone or download this repository.
2. Open a terminal or command prompt in the project directory.
3. Run the application:

```bash
python warehouse.py
```

---

## 🎮 How to Use & Controls

| Action                     | Description                                                                     |
| :------------------------- | :------------------------------------------------------------------------------ |
| **Click Grid Cell**        | Add or remove a shelf/obstacle (cannot modify Start or Delivery points).        |
| **FIND BEST ROUTE**        | Runs A\* and highlights the optimal path in yellow with performance stats.      |
| **RUN ROBOT / SHOW PATHS** | Animates the cell exploration phase followed by robot movement along the route. |
| **RANDOM OBSTACLES**       | Generates a new random warehouse obstacle layout.                               |
| **CLEAR ROUTE**            | Clears the current highlighted route and exploration overlay.                   |
| **RESET**                  | Restores the default warehouse shelf layout and robot position.                 |
| **Escape (Esc)**           | Toggle fullscreen mode.                                                         |

---

## 🗺️ Map Legend

- 🟩 **Green (S)**: Robot starting position
- 🟥 **Red (D)**: Delivery point destination
- ⬛ **Dark Slate**: Shelves / Obstacles (impassable walls)
- 🟨 **Yellow**: Optimal shortest route
- 🟦 **Light Blue**: Explored / reachable search area
- 🔵 **Blue Circle**: Robot avatar
