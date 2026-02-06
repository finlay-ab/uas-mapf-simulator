import numpy as np

# spatial manager for tracking UAS positions and checking conflicts
class SpatialManager:
    def __init__(self, safety_radius):
        self.safety_radius = safety_radius
        self.positions = {} 
        self.velocities = {} 
        self.states = {}    

    def update(self, uav_id, pos, vel, state):
        self.positions[uav_id] = pos
        self.velocities[uav_id] = vel
        self.states[uav_id] = state

    def check_conflicts(self, metrics):
        # get safety violations
        # for each uav pair, check distance
        uav_ids = list(self.positions.keys())
        conflict_detected = False
        
        for i in range(len(uav_ids)):
            for j in range(i + 1, len(uav_ids)):
                # calculate euclidean distance between pair
                uav_i = uav_ids[i]
                uav_j = uav_ids[j]
                dist = np.linalg.norm(self.positions[uav_i] - self.positions[uav_j])
                
                # check if distance is less than the diameter of the safety buffer
                metrics.record_separation(dist)
                
                if dist < (2 * self.safety_radius):
                    conflict_detected = True
        
        return conflict_detected