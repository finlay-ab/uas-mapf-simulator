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
