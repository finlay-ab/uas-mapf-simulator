import numpy as np

from src.policies.base import MAPFPolicy

class PolicyWrapper(MAPFPolicy):
    # abstract class for joining policies
    def __init__(self, base_policy: MAPFPolicy, active:bool=True):
        super().__init__(
            base_policy.grid_map,
            base_policy.max_speed,
            base_policy.safety_radius,
            base_policy.drone_radius,
        )
        self.base_policy = base_policy
        self.active = active

    def set_active(self, is_active:bool):
        self.active = is_active

    def plan_path(self, start, goal, spatial_manager=None):
        return self.base_policy.plan_path(start, goal, spatial_manager)

    def plan_path_to_waypoint(self, start, goal, spatial_manager=None):
        return self.base_policy.plan_path_to_waypoint(start, goal, spatial_manager)

    def get_velocity(self, name, pos, target, spatial_manager):
        return self.base_policy.get_velocity(name, pos, target, spatial_manager)
    