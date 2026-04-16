import sys
import os
import math
import time 

_SRC = os.path.dirname(os.path.abspath(__file__))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from simulator.simulator import Simulator
from planner.planner import Planner as plan_path
from controller.controller import Controller
from gui.gui import GUI


def grid_to_world(cell, scale):
    row, col = cell
    return [col * scale, row * scale]


def smooth_path(path):
    if len(path) < 3:
        return path

    smooth = [path[0]]

    for i in range(1, len(path) - 1):
        prev = smooth[-1]
        curr = path[i]
        nxt = path[i + 1]

        if (prev[0] - curr[0], prev[1] - curr[1]) == \
           (curr[0] - nxt[0], curr[1] - nxt[1]):
            continue

        smooth.append(curr)

    smooth.append(path[-1])
    return smooth


def inflate_obstacles(grid, padding=1):
    inflated = grid.copy()
    rows, cols = grid.shape

    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                for dr in range(-padding, padding + 1):
                    for dc in range(-padding, padding + 1):
                        rr = r + dr
                        cc = c + dc

                        if 0 <= rr < rows and 0 <= cc < cols:
                            inflated[rr, cc] = 1

    return inflated


# obstacle detection
def is_collision_nearby(pos, grid, scale):
    row = int(pos[1] / scale)
    col = int(pos[0] / scale)

    rows, cols = grid.shape

    for dr in range(-1, 2):
        for dc in range(-1, 2):
            r = row + dr
            c = col + dc

            if 0 <= r < rows and 0 <= c < cols:
                if grid[r, c] == 1:
                    half = scale / 2.0

                    obs_x = c * scale + half
                    obs_y = r * scale + half

                    dx = obs_x - pos[0]
                    dy = obs_y - pos[1]

                    if math.hypot(dx, dy) < 1.0:
                        return True

    return False


def main():
    gui = GUI()
    start, goal, grid = gui.run()

    if start is None or goal is None:
        return

    shortest_path = plan_path(start, goal, grid)

    inflated_grid = inflate_obstacles(grid, padding=1)
    safe_path = plan_path(start, goal, inflated_grid)

    if not safe_path:
        print("No safe path found")
        return

    safe_path = smooth_path(safe_path)

    scale = Simulator.CELL_SIZE
    sim = Simulator()

    start_world = grid_to_world(start, scale)
    goal_world  = grid_to_world(goal, scale)

    sim.load_robot(start_world)
    sim.add_obstacles_from_grid(grid)

    sim.mark_point(start_world, [0, 1, 0, 1])
    sim.mark_point(goal_world,  [1, 0, 0, 1])

    if shortest_path:
        shortest_world = [grid_to_world(p, scale) for p in shortest_path]
        sim.draw_path_lines(shortest_world)

    safe_world = [grid_to_world(p, scale) for p in safe_path]
    sim.draw_path_lines(safe_world)

    ctrl = Controller(sim.robot)

    # cooldown
    last_replan_time = 0
    REPLAN_COOLDOWN = 2.0

    while True:
        pos, yaw = sim.get_robot_pose()

        current_time = time.time()

        if is_collision_nearby(pos, grid, scale) and (current_time - last_replan_time > REPLAN_COOLDOWN):
            print(" Obstacle detected and Replanning")

            ctrl.stop()

            current_cell = (
                int(pos[1] / scale),
                int(pos[0] / scale)
            )

            safe_path = plan_path(current_cell, goal, inflated_grid)

            if not safe_path:
                print("No path found after replanning")
                break

            safe_path = smooth_path(safe_path)

            safe_world = [grid_to_world(p, scale) for p in safe_path]

            sim.draw_path_lines(safe_world)

            last_replan_time = current_time

        dist = ctrl.move_to(pos, yaw, safe_world)

        if dist < 0.3:
            ctrl.stop()
            print("Goal reached")

            while True:
                sim.step()

        sim.step()


if __name__ == "__main__":
    main()