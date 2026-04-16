import sys
import os
import math

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


def main():
    gui = GUI()
    start, goal, grid = gui.run()

    if start is None or goal is None:
        return

    path = plan_path(start, goal, grid)

    if not path:
        print("No path found")
        return

    path = smooth_path(path)

    scale = Simulator.CELL_SIZE
    sim = Simulator()

    start_world = grid_to_world(start, scale)
    goal_world  = grid_to_world(goal, scale)

    sim.load_robot(start_world)
    sim.add_obstacles_from_grid(grid)

    sim.mark_point(start_world, [0, 1, 0, 1])
    sim.mark_point(goal_world,  [1, 0, 0, 1])

    world_path = [grid_to_world(p, scale) for p in path]
    sim.draw_path_lines(world_path)

    ctrl = Controller(sim.robot)

    while True:
        pos, yaw = sim.get_robot_pose()

        dist = ctrl.move_to(pos, yaw, world_path)

        if dist < 0.3:
            ctrl.stop()
            print("Goal reached")

            while True:
                sim.step()

        sim.step()


if __name__ == "__main__":
    main()