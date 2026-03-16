import numpy as np

from src.physics import Velocity 
from src.policies.base import MAPFPolicy

class GreedyPolicy(MAPFPolicy):
    def __init__(self, max_speed):
        self.max_speed = max_speed

    def get_velocity(self, name, pos, target, spatial_manager):
        direction = target - pos
        distance = np.linalg.norm(direction)

        # if arrived at target
        if distance < 0.5:
            return Velocity(0.0, 0.0)
        
        vel_vector = (direction / distance) * self.max_speed
        
        return Velocity(vel_vector[0], vel_vector[1])
