import heapq
import math
import numpy as np


def _heuristic(a: tuple, b: tuple) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


_DIRECTIONS = [
    (1,  0, 1.0), (-1,  0, 1.0), (0,  1, 1.0), (0, -1, 1.0),
    (1,  1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1,-1, math.sqrt(2)),
]


def compute_obstacle_cost(grid):
    rows, cols = grid.shape
    cost = np.zeros_like(grid, dtype=float)

    obstacle_cells = np.argwhere(grid == 1)

    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                cost[r, c] = 100.0
            else:
                min_dist = min(
                    [math.hypot(r - orow, c - ocol) for orow, ocol in obstacle_cells]
                    or [10]
                )
                cost[r, c] = 1.0 / (min_dist + 1e-5)

    return cost


def plan_path(start, goal, grid):
    rows, cols = grid.shape
    obstacle_cost = compute_obstacle_cost(grid)

    open_list = []
    heapq.heappush(open_list, (0.0, start))

    came_from = {}
    g_cost = {start: 0.0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            return path[::-1]

        for dr, dc, step_cost in _DIRECTIONS:
            neighbor = (current[0] + dr, current[1] + dc)

            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue

            if grid[neighbor] == 1:
                continue

            if dr != 0 and dc != 0:
                if grid[current[0] + dr, current[1]] == 1 or \
                   grid[current[0], current[1] + dc] == 1:
                    continue

            penalty = obstacle_cost[neighbor] * 5.0

            tentative_g = g_cost[current] + step_cost + penalty

            if tentative_g < g_cost.get(neighbor, math.inf):
                g_cost[neighbor] = tentative_g
                f = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_list, (f, neighbor))
                came_from[neighbor] = current

    return []


Planner = plan_path