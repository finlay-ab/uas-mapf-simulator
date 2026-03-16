
# abstract base class for MAPF policies
class MAPFPolicy:
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def plan_path(self, start, goal, drone_radius, safety_radius):
        raise NotImplementedError("planners must implement the plan_path method")
    