import numpy as np

from src.policies.base import MAPFPolicy

class PolicyWrapper(MAPFPolicy):
    # abstract class for joining policies
    def __init__(self, base_policy: MAPFPolicy, active:bool=True):
        self.base_policy = base_policy
        self.active = active

    def set_active(self, is_active:bool):
        self.active = is_active

    def get_velocity(self, name, pos, target, spartial_manager):
        return self.base_policy.get_velocity(name, pos, target, spartial_manager)
    