import numpy as np

from src.physics import LocalPosition, Velocity
from src.policies.base import MAPFPolicy

class GreedyPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed, safety_radius=1.0, drone_radius=0.5):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)

    def plan_path(self, start: LocalPosition, goal: LocalPosition, spatial_manager=None, uav_id=None):
        del spatial_manager
        del uav_id
        return [start, goal]

    def get_velocity(self, name, pos, target, spatial_manager):
        # doesnt use name or spartial
        del name
        del spatial_manager

        # get direction and distance to targ
        pos_array = self._position_array(pos)
        target_array = self._position_array(target)
        direction = target_array - pos_array
        distance = np.linalg.norm(direction)

        # check if at target
        if distance < 0.01:
            return Velocity(0.0, 0.0)

        # get max speed towards target
        unit_direction = direction / distance
        speed = min(self.max_speed, distance)
        velocity = unit_direction * speed
        return Velocity(float(velocity[0]), float(velocity[1]))