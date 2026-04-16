import math
import pybullet as p
from typing import List, Tuple


def _discover_wheel_joints(robot_id: int) -> Tuple[List[int], List[int]]:
    left_joints, right_joints = [], []
    n = p.getNumJoints(robot_id)

    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")

        if any(lw in name for lw in ("front_left", "rear_left")):
            left_joints.append(i)
        elif any(rw in name for rw in ("front_right", "rear_right")):
            right_joints.append(i)

    if len(left_joints) != 2 or len(right_joints) != 2:
        left_joints  = [2, 4]
        right_joints = [3, 5]

    return left_joints, right_joints


class Controller:
    MAX_LINEAR = 2.0
    FORCE      = 50.0

    def __init__(self, robot_id: int):
        self.robot = robot_id
        self.left_joints, self.right_joints = _discover_wheel_joints(robot_id)
        print(f"[Controller] Left joints: {self.left_joints}, Right joints: {self.right_joints}")

    def move_to(self, current_pos, current_yaw, path, lookahead_dist=0.8):
        """
        Correct Pure Pursuit Controller
        """

        # Find closest point
        closest_idx = 0
        min_dist = float("inf")

        for i, pt in enumerate(path):
            dx = pt[0] - current_pos[0]
            dy = pt[1] - current_pos[1]
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Lookahead point
        lookahead_idx = min(closest_idx + 2, len(path) - 1)
        target = path[lookahead_idx]

        # Transform to robot frame
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]

        angle_to_target = math.atan2(dy, dx)
        alpha = angle_to_target - current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    
        L = lookahead_dist
        curvature = (2 * math.sin(alpha)) / L

        linear_vel = self.MAX_LINEAR
        angular_vel = curvature * linear_vel

        left_vel  = linear_vel - angular_vel
        right_vel = linear_vel + angular_vel

        self._set_wheels(left_vel, right_vel)

        return math.hypot(dx, dy)

    def stop(self):
        self._set_wheels(0.0, 0.0)

    def _set_wheels(self, left_vel: float, right_vel: float):
        for j in self.left_joints:
            p.setJointMotorControl2(
                self.robot, j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=self.FORCE,
            )

        for j in self.right_joints:
            p.setJointMotorControl2(
                self.robot, j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=self.FORCE,
            )