import numpy as np
from src.entities import UAVState
from src.schemas import UAV_SEGMENT, AirspaceType

# spatial manager for tracking UAS positions and checking conflicts
class SpatialManager:
    def __init__(self, safety_radius, metrics=None, env=None, grid_map=None, origin=None):
        self.safety_radius = safety_radius
        self.metrics = metrics  
        self.env = env
        self.grid_map = grid_map
        self.origin = origin
        
        # position/state snapshots for event boundaries
        self.positions = {}
        self.states = {}
        self.radii = {}
        self.velocities = {}
        
        # uav segments
        self.active_segments = {} 
        
        # track collision/ violations 
        # (set to avoid duplicate pairs)
        self.active_collision_pairs = set()
        self.active_violation_pairs = set()
        
        # saftey margin = +10% of radius
        self.safety_margin = 0.10
        
        # landing reservations
        self.ground_zone_owners = {}
        self.uav_ground_zone = {}

        # time index reservations
        self.reservations = {}
        self.edge_reservations = {}

    # --------- depot reservation -----------------
    def _to_array(self, value):
        if hasattr(value, "as_array"):
            return np.asarray(value.as_array(), dtype=float)
        return np.asarray(value, dtype=float)

    def _is_depot_internal_state(self, state):
        return state in (UAVState.IDLE_DEPOT, UAVState.TAKEOFF, UAVState.LANDING)

    # returns a rounded x and y for the co orinates for the center of a depot 
    def _ground_zone_key(self, center):
        return (round(float(center[0]), 2), round(float(center[1]), 2))
    
    # makes sure uavs cant takeoff/ land in same place as another
    def reserve_ground_zone(self, uav_id, center):
        center = self._to_array(center)
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
        # check if area around center is clear for takeoff/landing
        center = self._to_array(center)
        for other_id in self.positions.keys():
            if other_id == uav_id:
                continue

            other_state = self.states.get(other_id)
            if self._is_depot_internal_state(other_state):
                continue

            # if flying, check against segment; else use snapshot
            other_pos = self._get_current_position(other_id)
            if other_pos is None:
                continue

            dist = np.linalg.norm(other_pos - center)
            if dist < clearance_radius:
                return False

        return True

    # ------- time space reservations --------------------------
    def is_reserved(self, cell, time_idx, uav_id=None):
        # check if cell is reserved at a given time
        key = (int(cell[0]), int(cell[1]), int(time_idx))
        owner = self.reservations.get(key)
        if owner is None:
            return False
        if uav_id is not None and owner == uav_id:
            return False
        return True

    # reserve cell for uav at given time
    def reserve_cell(self, uav_id, cell, time_idx):
        key = (int(cell[0]), int(cell[1]), int(time_idx))
        owner = self.reservations.get(key)
        if owner is None or owner == uav_id:
            self.reservations[key] = uav_id
            if self.metrics is not None and hasattr(self.metrics, "record_reservation_attempt"):
                self.metrics.record_reservation_attempt(True)
            return True
        if self.metrics is not None and hasattr(self.metrics, "record_reservation_attempt"):
            self.metrics.record_reservation_attempt(False)
        return False
    
    def is_edge_reserved(self, from_cell, to_cell, time_idx, uav_id=None):
        key = ( int(from_cell[0]), int(from_cell[1]), int(to_cell[0]), int(to_cell[1]), int(time_idx))
        owner = self.edge_reservations.get(key)
        if owner is None:
            return False
        if uav_id is not None and owner == uav_id:
            return False
        return True

    def reserve_edge(self, uav_id, from_cell, to_cell, time_idx):
        key = ( int(from_cell[0]), int(from_cell[1]), int(to_cell[0]), int(to_cell[1]), int(time_idx))
        owner = self.edge_reservations.get(key)
        if owner is None or owner == uav_id:
            self.edge_reservations[key] = uav_id
            return True
        return False

    def is_swap_conflict(self, from_cell, to_cell, time_idx, uav_id=None):
        return self.is_edge_reserved(to_cell, from_cell, time_idx, uav_id=uav_id)

    # clears all reservations owned by given uav
    def clear_reservations(self, uav_id):
        # check id
        if uav_id is None:
            return
    
        # remove keys
        remove_keys = [key for key, owner in self.reservations.items() if owner == uav_id]
        for key in remove_keys:
            del self.reservations[key]
        
        # remove edges
        remove_edge_keys = [key for key, owner in self.edge_reservations.items() if owner == uav_id]
        for key in remove_edge_keys:
            del self.edge_reservations[key]

    # removes old reservations 
    def expire_stale_reservations(self, current_time_idx):
        # get keys from past
        stale_keys = [key for key in self.reservations if key[2] < current_time_idx]
        for key in stale_keys:
            del self.reservations[key]
        # get edges from past
        stale_edges = [key for key in self.edge_reservations if key[4] < current_time_idx]
        for key in stale_edges:
            del self.edge_reservations[key]

    # ------------- conflicts --------------
    def check_conflicts(self, metrics, current_time):
        uav_ids = list(self.positions.keys())
        conflict_detected = False
        
        for i in range(len(uav_ids)):
            for j in range(i + 1, len(uav_ids)):
                a = uav_ids[i]
                b = uav_ids[j]
                
                # ignore uavs inside depot
                state_a = self.states.get(a)
                state_b = self.states.get(b)
                if self._is_depot_internal_state(state_a) or self._is_depot_internal_state(state_b):
                    continue

                # get current pos
                pos_a = self._get_current_position(a, current_time)
                pos_b = self._get_current_position(b, current_time)
                if pos_a is None or pos_b is None:
                    continue
                    
                dist = np.linalg.norm(pos_a - pos_b)
                
                # record minimum separation
                metrics.record_separation(dist)
                
                # get radii from registered segments
                radius_a = self.radii.get(a, self.safety_radius)
                radius_b = self.radii.get(b, self.safety_radius)
                
                # collision if centers closer than sum of radii
                collision_distance = radius_a + radius_b
                
                # safety threshold
                safety_threshold = collision_distance * (1.0 + self.safety_margin)
                
                pair = tuple(sorted([a, b]))
                
                # hard collision
                if dist < collision_distance:
                    if pair not in self.active_collision_pairs:
                        metrics.record_collision(dist, a, b)
                        self.active_collision_pairs.add(pair)
                    conflict_detected = True
                else:
                    if pair in self.active_collision_pairs:
                        self.active_collision_pairs.discard(pair)
                
                # near miss
                if collision_distance <= dist < safety_threshold:
                    if pair not in self.active_violation_pairs:
                        metrics.record_safety_violation(dist, a, b)
                        self.active_violation_pairs.add(pair)
                else:
                    if pair in self.active_violation_pairs:
                        self.active_violation_pairs.discard(pair)
        
        return conflict_detected
    
    def _get_current_position(self, uav_id, current_time=None):
        if uav_id in self.active_segments:
            segment = self.active_segments[uav_id]
            if current_time is not None:
                pos_array = self._position_at(segment, current_time)
                if pos_array is not None:
                    return pos_array
            return self._to_array(segment.end_position)
        return self.positions.get(uav_id)

    def update_position_snapshot(self, uav_id, pos, state):
        # update snapshot only at event boundaries: landing, segment end, airspace transfer
        self.positions[uav_id] = self._to_array(pos)
        self.states[uav_id] = state
        self.velocities[uav_id] = np.array([0.0, 0.0], dtype=float)

    # update only the state, dont touch position or velocity (caller may have an active segment)
    def update_state(self, uav_id, state):
        self.states[uav_id] = state

    def register_segment(self, uav_id, segment: UAV_SEGMENT):
        # register active motion segment 
        self.active_segments[uav_id] = segment
        # radius comes from segment only
        self.radii[uav_id] = float(segment.radius)
        self.velocities[uav_id] = self._to_array(segment.velocity)

    def deregister_segment(self, uav_id, final_position):
        # remove segment when segment ends and store final pos as snapshot
        segment = self.active_segments.pop(uav_id, None)
        self.update_position_snapshot(uav_id, final_position, self.states.get(uav_id))
        self.velocities[uav_id] = np.array([0.0, 0.0], dtype=float)
        
        # record precomputed collision/violation data at the event boundary
        if self.metrics is not None and segment is not None:
            for other_id, distance in segment.predicted_collisions:
                self.metrics.record_separation(distance)
                self.metrics.record_collision(distance, uav_id, other_id)

            for other_id, distance in segment.predicted_violations:
                self.metrics.record_separation(distance)
                self.metrics.record_safety_violation(distance, uav_id, other_id)

    def _position_at(self, segment: UAV_SEGMENT, time: float):
        # linear interpolation of segment
        # p(t) = start + (t - t_start) / (t_end - t_start) * (end - start)
        t0 = segment.start_time
        t1 = segment.end_time
        if time < t0 or time > t1:
            return None
        duration = t1 - t0
        if duration <= 0:
            return self._to_array(segment.end_position)
        alpha = (time - t0) / duration
        start = self._to_array(segment.start_position)
        end = self._to_array(segment.end_position)
        return start + alpha * (end - start)

    def _segment_min_distance(self, seg_a: UAV_SEGMENT, seg_b: UAV_SEGMENT):
        # compute the true minimum distance between two linearly moving segments
        # using continuous collision detection

        t_start = max(seg_a.start_time, seg_b.start_time)
        t_end = min(seg_a.end_time, seg_b.end_time)
        if t_start > t_end:
            return None
        
        # get positions and vel at overlap 
        pa_start = self._position_at(seg_a, t_start)
        pb_start = self._position_at(seg_b, t_start)
        if pa_start is None or pb_start is None:
            return None
        
        # compute velocities from segment endpoints
        dur_a = seg_a.end_time - seg_a.start_time
        dur_b = seg_b.end_time - seg_b.start_time
        if dur_a <= 0 or dur_b <= 0:
            # stationary segments; check start positions
            distance = float(np.linalg.norm(pa_start - pb_start))
            return distance
        
        start_a = self._to_array(seg_a.start_position)
        end_a = self._to_array(seg_a.end_position)
        start_b = self._to_array(seg_b.start_position)
        end_b = self._to_array(seg_b.end_position)
        
        va = (end_a - start_a) / dur_a
        vb = (end_b - start_b) / dur_b
        
        # relative position and velocity in overlap window
        r0 = pa_start - pb_start  # relative position at t_start
        vr = va - vb  # relative velocity
        
        # maths needed
        # d^2 from time since t_start
        # d²(tau) = |r0 + vr*tau|² = r0·r0 + 2*tau*(r0·vr) + tau²*(vr·vr)
        # d(d²)/d(tau) = 2*(r0·vr) + 2*tau*(vr·vr) = 0
        # tau_min = -(r0·vr) / (vr·vr)
        
        vr_mag_sq = np.dot(vr, vr)
        
        if vr_mag_sq < 1e-12:
            # same velocity; distance is constant
            distance = float(np.linalg.norm(r0))
            return distance
        
        # analytical time of closest approach
        tau_min = -np.dot(r0, vr) / vr_mag_sq
        
        # clamp to valid window [0, t_end - t_start]
        overlap_duration = t_end - t_start
        tau_min = np.clip(tau_min, 0.0, overlap_duration)
        
        # evaluate distance at critical point
        r_at_min = r0 + vr * tau_min
        min_distance = float(np.linalg.norm(r_at_min))
        
        return min_distance

    # returns if segment collides with static obstacles
    def _segment_hits_obstacle(self, candidate: UAV_SEGMENT):
        if self.grid_map is None or self.origin is None:
            return False, None

        # get grid co-ordinate
        origin_array = self._to_array(self.origin)
        start_local = self._to_array(candidate.start_position) - origin_array
        end_local = self._to_array(candidate.end_position) - origin_array

        dx = end_local[0] - start_local[0]
        dy = end_local[1] - start_local[1]
        length = float(np.hypot(dx, dy))

        # if segment is only 1 cell
        if length < 1e-6:
            return False, None

        # check along line segemnt for obstacles
        n_samples = max(2, int(np.ceil(length / self.grid_map.resolution)) + 1)
        for i in range(n_samples):
            alpha = i / (n_samples - 1)
            x = start_local[0] + alpha * dx
            y = start_local[1] + alpha * dy
            gx, gy = self.grid_map.world_to_grid_point(x, y, clamp=True)
            if self.grid_map.grid[gx, gy] == AirspaceType.OBSTACLE.value:
                return True, (gx, gy)

        return False, None

    def check_segment_safety(self, uav_id, candidate: UAV_SEGMENT):
        # pre-compute collision/violation data for this segment
        # returns true if the segment is  safe
        candidate.predicted_collisions.clear()
        candidate.predicted_violations.clear()

        is_safe = True

        hit_obstacle, obstacle_cell = self._segment_hits_obstacle(candidate)
        
        if hit_obstacle:
            # add collision to metrics
            obstacle_id = f"OBSTACLE@{obstacle_cell}"
            collision_time = 0.0
            
            candidate.predicted_collisions.append((obstacle_id, collision_time))
            is_safe = False

        # dynamic check
        for other_id, other in self.active_segments.items():
            if other_id == uav_id:
                continue
            
            # skip if other uav in depot state
            other_state = self.states.get(other_id)
            if self._is_depot_internal_state(other_state):
                continue
            
            # skip if time windows dont overlap
            if candidate.end_time < other.start_time or candidate.start_time > other.end_time:
                continue
            
            min_distance = self._segment_min_distance(candidate, other)
            if min_distance is None:
                continue

            radius_a = float(candidate.radius)
            radius_b = float(other.radius)
            collision_distance = radius_a + radius_b
            safety_threshold = collision_distance * (1.0 + self.safety_margin)

            if min_distance < collision_distance:
                candidate.predicted_collisions.append((other_id, min_distance))
                is_safe = False
            elif min_distance < safety_threshold:
                candidate.predicted_violations.append((other_id, min_distance))

        return is_safe