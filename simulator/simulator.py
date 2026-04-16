import pybullet as p
import pybullet_data
import time


class Simulator:

    CELL_SIZE = 0.7

    def __init__(self):
        self._client = p.connect(p.GUI)


        p.configureDebugVisualizer(p.COV_ENABLE_GUI,           0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)


        p.setTimeStep(1.0 / 240.0)

        self.plane    = p.loadURDF("plane.urdf")
        self.robot    = None
        self.obstacles: list = []
        self._path_lines: list = []



    def load_robot(self, start_pos):
        
        spawn = [start_pos[0], start_pos[1], 0.15]
        self.robot = p.loadURDF("husky/husky.urdf", spawn,
                                flags=p.URDF_USE_INERTIA_FROM_FILE)

        for _ in range(50):
            p.stepSimulation()
        return self.robot




    def add_obstacles_from_grid(self, grid):
       
        self.obstacles = []
        rows, cols = grid.shape
        half = self.CELL_SIZE / 2.0

        for r in range(rows):
            for c in range(cols):
                if grid[r, c] == 1:

                    wx = c * self.CELL_SIZE + half
                    wy = r * self.CELL_SIZE + half

                    col_shape = p.createCollisionShape(
                        p.GEOM_BOX,
                        halfExtents=[half * 0.95, half * 0.95, 0.4],
                    )
                    vis_shape = p.createVisualShape(
                        p.GEOM_BOX,
                        halfExtents=[half * 0.95, half * 0.95, 0.4],
                        rgbaColor=[0.25, 0.25, 0.25, 1.0],
                    )
                    body = p.createMultiBody(
                        baseMass=0,                 
                        baseCollisionShapeIndex=col_shape,
                        baseVisualShapeIndex=vis_shape,
                        basePosition=[wx, wy, 0.4],
                    )
                    self.obstacles.append(body)

    def mark_point(self, world_xy, color_rgba):

        vis = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.18, rgbaColor=color_rgba
        )
        p.createMultiBody(
            baseVisualShapeIndex=vis,
            basePosition=[world_xy[0], world_xy[1], 0.18],
        )

    def draw_path_lines(self, world_path: list):
        
        for lid in self._path_lines:
            p.removeUserDebugItem(lid)
        self._path_lines = []

        color = [0.0, 0.8, 1.0]   
        for i in range(len(world_path) - 1):
            a = [world_path[i][0],     world_path[i][1],     0.1]
            b = [world_path[i+1][0],   world_path[i+1][1],   0.1]
            lid = p.addUserDebugLine(a, b, color, lineWidth=2.5)
            self._path_lines.append(lid)


    def get_robot_pose(self):

        pos, orn = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return pos, yaw

    def step(self):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)