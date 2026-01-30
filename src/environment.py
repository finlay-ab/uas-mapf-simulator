import numpy as np

# metrics for testing and evaluation
class Metrics:
    def __init__(self):
        self.completed_deliveries = 0
        self.total_violations = 0
        self.flight_paths = {} 

    def record_path(self, uav_id, pos):
        if uav_id not in self.flight_paths: 
            self.flight_paths[uav_id] = []
        self.flight_paths[uav_id].append(pos.copy())

# spatial manager for tracking UAS positions and checking conflicts
class SpatialManager:
    def __init__(self, safety_radius):
        self.safety_radius = safety_radius
        self.positions = {} 
        self.velocities = {} 
        self.states = {}    
        self.violations = 0 

    def update(self, uav_id, pos, vel, state):
        self.positions[uav_id] = pos
        self.velocities[uav_id] = vel
        self.states[uav_id] = state

    def check_conflicts(self):
        # get safety violations
        # for each uav pair, check distance
        uav_ids = list(self.positions.keys())
        conflict_detected = False
        
        for i in range(len(uav_ids)):
            for j in range(i + 1, len(uav_ids)):
                # calculate euclidean distance between pair
                dist = np.linalg.norm(self.positions[uav_ids[i]] - self.positions[uav_ids[j]])
                
                # check if distance is less than the diameter of the safety buffer
                if dist < (2 * self.safety_radius):
                    self.violations += 1
                    conflict_detected = True
        
        return conflict_detected