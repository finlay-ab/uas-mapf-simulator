import numpy as np
from .base import MAPFPolicy

class GreedyPolicy(MAPFPolicy):
    def __init__(self, max_speed):
        self.max_speed = max_speed

    def get_velocity(self, name, pos, target, spatial_manager):
        direction = target - pos
        distance = np.linalg.norm(direction)

        # if arrived at target
        if distance < 0.5:
            return np.array([0.0, 0.0, 0.0])
        
        velocity = (direction / distance) * self.max_speed
        return velocity