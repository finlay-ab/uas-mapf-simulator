import numpy as np

# metrics for testing and evaluation
class Metrics:
    def __init__(self):
        self.completed_deliveries = 0
        self.total_violations = 0
        self.flight_paths = {} 

    def record_path(self, name, pos):
        if name not in self.flight_paths: self.flight_paths[name] = []
        self.flight_paths[name].append(pos.copy())

# spatial manager for tracking UAS positions and checking conflicts
class SpatialManager:
    def __init__(self, safety_radius):
        self.safety_radius = safety_radius
        self.positions = {} 
        self.violations = 0 

    def update(self, name, pos):
        self.positions[name] = pos

    def check_conflicts(self):
        # get safty violations
        # for each agent pair, check distance
        agents = list(self.positions.keys())
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                dist = np.linalg.norm(self.positions[agents[i]] - self.positions[agents[j]])
                if dist < (2 * self.safety_radius):
                    self.violations += 1
                    return True
        return False