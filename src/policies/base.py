import numpy as np

from src.physics import Velocity

# abstract base class for MAPF policies
class MAPFPolicy:
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5):
        self.grid_map = grid_map
        self.max_speed = max_speed
        self.safety_radius = safety_radius
        self.drone_radius = drone_radius

    def plan_path(self, start, goal):
        raise NotImplementedError("planners must implement the plan_path method")

    # used to replan when off path 
    def plan_path_to_waypoint(self, start, goal):
        return self.plan_path(start, goal)

    
    def get_velocity(self, name, pos, target, spatial_manager):
        raise NotImplementedError("planners must implement the get_velocity method")