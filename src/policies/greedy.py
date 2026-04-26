import numpy as np

from src.physics import LocalPosition, Velocity
from src.policies.base import MAPFPolicy

class GreedyPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed, safety_radius=1.0, drone_radius=0.5):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)

    def plan_path(self, start: LocalPosition, goal: LocalPosition, spatial_manager=None):
        return [start, goal]
    

