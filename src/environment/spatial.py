import numpy as np
from src.entities import UAVState

# spatial manager for tracking UAS positions and checking conflicts
class SpatialManager:
    def __init__(self, safety_radius):
        self.safety_radius = safety_radius
        self.positions = {} 
        self.velocities = {} 
        self.states = {}    
        self.ground_zone_owners = {}
        self.uav_ground_zone = {}

    def update(self, uav_id, pos, vel, state):
        self.positions[uav_id] = pos
        self.velocities[uav_id] = vel
        self.states[uav_id] = state

    def _is_depot_internal_state(self, state):
        return state in (UAVState.IDLE_DEPOT, UAVState.TAKEOFF, UAVState.LANDING)

    # returns a rounded x and y for the co orinates for the center of a depot 
    def _ground_zone_key(self, center):
        return (round(float(center[0]), 2), round(float(center[1]), 2))

    # makes sure uavs cant takeoff/ land in same place as another
    def reserve_ground_zone(self, uav_id, center):
        zone = self._ground_zone_key(center)
        owner = self.ground_zone_owners.get(zone)
        if owner is None or owner == uav_id:
            self.ground_zone_owners[zone] = uav_id
            self.uav_ground_zone[uav_id] = zone
            return True
        return False

    def release_ground_zone(self, uav_id):
        zone = self.uav_ground_zone.pop(uav_id, None)
        if zone is None:
            return

        if self.ground_zone_owners.get(zone) == uav_id:
            del self.ground_zone_owners[zone]

    def is_takeoff_landing_clear(self, uav_id, center, clearance_radius):
        for other_id, other_pos in self.positions.items():
            if other_id == uav_id:
                continue

            other_state = self.states.get(other_id)
            if self._is_depot_internal_state(other_state):
                continue

            dist = np.linalg.norm(other_pos - center)
            if dist < clearance_radius:
                return False

        return True

    def check_conflicts(self, metrics):
        # get safety violations
        # for each uav pair, check distance
        uav_ids = list(self.positions.keys())
        conflict_detected = False
        
        for i in range(len(uav_ids)):
            for j in range(i + 1, len(uav_ids)):
                # ignores uavs "insdie" depot
                state_a = self.states.get(uav_ids[i])
                state_b = self.states.get(uav_ids[j])
                if self._is_depot_internal_state(state_a) or self._is_depot_internal_state(state_b):
                    continue

                dist = np.linalg.norm(self.positions[uav_ids[i]] - self.positions[uav_ids[j]])
                metrics.record_separation(dist)
                
                if dist < (2 * self.safety_radius):
                    conflict_detected = True
        
        return conflict_detected