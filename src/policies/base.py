import numpy as np

from src.physics import Velocity, LocalPosition

# abstract base class for MAPF policies
class MAPFPolicy:
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5):
        self.grid_map = grid_map
        self.max_speed = max_speed
        self.safety_radius = safety_radius
        self.drone_radius = drone_radius

    # helper to convert pos into arr
    @staticmethod
    def _position_array(position):
        if hasattr(position, "as_array"):
            return np.array(position.as_array(), dtype=float)
        return np.array(position, dtype=float)

    def plan_path(self, start: LocalPosition, goal: LocalPosition, spatial_manager=None, uav_id=None):
        raise NotImplementedError("planners must implement the plan_path method")

    # used to replan when off path 
    def plan_path_to_waypoint(self, start: LocalPosition, goal: LocalPosition, spatial_manager=None, uav_id=None):
        return self.plan_path(start, goal, spatial_manager, uav_id)

    
    def get_velocity(self, name: str, pos: LocalPosition, target: LocalPosition, spatial_manager=None):
        raise NotImplementedError("planners must implement the get_velocity method")