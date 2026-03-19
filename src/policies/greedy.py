import numpy as np

from src.physics import Velocity
from src.policies.base import MAPFPolicy

class GreedyPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed, safety_radius=1.0, drone_radius=0.5):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)

    def plan_path(self, start, goal):
        start_array = np.array(start, dtype=float)
        goal_array = np.array(goal, dtype=float)
        return [start_array, goal_array]

    def get_velocity(self, name, pos, target, spatial_manager):
        del name
        del spatial_manager

        pos_array = np.array(pos, dtype=float)
        target_array = np.array(target, dtype=float)
        direction = target_array - pos_array
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return Velocity(0.0, 0.0)

        unit_direction = direction / distance
        speed = min(self.max_speed, distance)
        velocity = unit_direction * speed
        return Velocity(float(velocity[0]), float(velocity[1]))
