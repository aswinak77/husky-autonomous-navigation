# Husky Autonomous Navigation (PyBullet)

## Overview

This project demonstrates autonomous navigation of a Husky robot in a simulated environment using PyBullet. The robot plans a path from a start point to a goal while avoiding obstacles on a grid-based map.

The system combines path planning, obstacle avoidance, and real-time control to move the robot smoothly through the environment.

---

## Features

* Grid-based GUI for selecting start and goal points
* A* path planning algorithm
* Obstacle inflation for safe navigation
* Path replanning when obstacles are detected

## How It Works

1. The user selects a start and goal point in the GUI.
2. The planner computes the shortest path.
3. User can visulaize the robot follows the path in PyBullet.
4. If an obstacle is detected nearby, the system replans the path.

### System Requirements

* Python 3.8 or higher
* Linux / Ubuntu recommended

### Python Dependencies

*If you prefer to run directly using system Python, install compatible versions:

 pip3 install numpy==1.24.4
 pip3 install pybullet==3.2.5
 pip3 install opencv-python

Or using a virtual environment (recommended):

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

## How to Run

1. Activate virtual environment:

source venv/bin/activate

2. Run the program:

python main.py

3. In the GUI:

* Left click → set Start
* Left click again → set Goal
* Right click → reset

4. Simulation will start automatically
