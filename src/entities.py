from enum import auto, Enum
import numpy as np
import simpy
import logging
import json
from .physics import Velocity
from .environment.map import AirspaceType
from .schemas import PathRecoveryStrategy, PathRecoveryAction

# get log
log = logging.getLogger("UAS_Sim")

# enum to track state for metrics
class UAVState(Enum):
    IDLE_DEPOT = auto()
    TAKEOFF = auto()
    EN_ROUTE = auto()
    HOVER_WAIT = auto()
    DELIVERING = auto()
    RETURNING = auto()
    LANDING = auto()

# UAV entity which stores state and jobs
class UAV:
    def __init__(self, env, uav_id, depot_owner_id, policy, job_queue, uav_config_file, grid_map, sm, cfg):
        
        # set input vars
        self.env = env
        self.uav_id = uav_id
        self.depot_owner_id = depot_owner_id
        self.cfg = cfg

        self.depot = grid_map.get_depot_position(depot_owner_id)

        self.policy = policy
        self.job_queue = job_queue

        # open config file
        with open("config/uavs/" + uav_config_file + ".json", 'r') as f:
            self.uav_config = json.load(f)

        self.grid_map = grid_map
        self.sm = sm
        


        # set uav vars
        self.vel = Velocity(0.0, 0.0)

        # get depot pos from grid map
        self.pos = self.grid_map.get_depot_position(depot_owner_id)

        # start at depot
        self.state = UAVState.IDLE_DEPOT
        self.current_job = None

        # path planning
        self.current_path = []
        self.path_index = 0

        # path recovery tracking
        self.path_deviation_point = None # where UAV left the path
        self.recovery_attempts = 0 
        self.hover_start_time = None
        self.last_valid_pos = np.array(self.pos, dtype=float)
        self.skipped_waypoints = 0    
        self.job_start_time = None     

        # temp 
        depot_pos = self.grid_map.get_depot_position(depot_owner_id)
        if depot_pos is None:
            raise ValueError(f"Unknown depot id '{depot_owner_id}' for UAV {uav_id}")

        self.depot = np.array(depot_pos, dtype=float)
        self.pos = self.depot.copy()

        # start process
        # give state to sm
        if self.sm is not None:
            self.sm.update(self.uav_id, self.pos, self.vel, self.state)
        self.action = env.process(self.run())

    def wait_for_depot_clearance(self, phase):
        # check for spartial manager
        if self.sm is None:
            return

        # reserve space and wait for clearance
        while True:
            reserved = self.sm.reserve_ground_zone(self.uav_id, self.depot)
            clear = self.sm.is_takeoff_landing_clear(
                self.uav_id,
                self.depot,
                self.cfg.depot_operation_radius
            )

            # update sm to reeflect waiting state 
            self.sm.update(self.uav_id, self.pos, self.vel, self.state)

            # procceed
            if reserved and clear:
                return

            # cancel reservation to prevent deadlock with take offs
            if reserved and not clear:
                self.sm.release_ground_zone(self.uav_id)

            # log and wait before checking again
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} waiting for {phase} clearance.")
            yield self.env.timeout(self.cfg.depot_check_interval)

    def run(self):
        # UAV loop
        while True:
            # idle at depot (waiting for job)
            self.state = UAVState.IDLE_DEPOT
            if self.sm is not None:
                self.vel = Velocity(0.0, 0.0)
                self.sm.update(self.uav_id, self.pos, self.vel, self.state)
            self.current_job = yield self.job_queue.get()
            target = np.array(self.current_job['goal'], dtype=float)
            
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} assigned to job {self.current_job['id']}")

            # takeoff phase (2s timeout to simulate takeoff)
            self.state = UAVState.TAKEOFF
            self.vel = Velocity(0.0, 0.0)
            yield from self.wait_for_depot_clearance("takeoff")
            yield self.env.timeout(2.0) 
            if self.sm is not None:
                self.sm.release_ground_zone(self.uav_id)

            # en_route to target
            start_delivery = self.env.now
            self.state = UAVState.EN_ROUTE
            yield from self.execute_flight(target)
            
            # record delivery time 
            self.metrics.record_delivery_phase(self.env.now - start_delivery)

            # drop off package (3s timeout to simulate delivery/ drop off)
            self.state = UAVState.DELIVERING
            yield self.env.timeout(3.0) 
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} completed job {self.current_job['id']}")

            # return to depot
            start_return = self.env.now
            self.state = UAVState.RETURNING
            yield from self.execute_flight(self.depot)
            self.metrics.record_return_phase(self.env.now - start_return)
            
            # landing phase (2s timeout to simulate landing)
            self.state = UAVState.LANDING
            self.vel = Velocity(0.0, 0.0)
            yield from self.wait_for_depot_clearance("landing")
            yield self.env.timeout(2.0)
            if self.sm is not None:
                self.sm.release_ground_zone(self.uav_id)
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} recovered at depot.")

            # clear job
            self.current_job = None

    def calculate_point_to_path_distance(self, point, path):
        # calcuate the shortest distnace to an edge on the path
        
        # if path is empty or only 1 point
        if len(path) == 0:
            return float('inf')
        elif len(path) == 1:
            return np.linalg.norm(point - path[0])

        min_dist = float('inf')

        # for each seg
        for i in range(len(path) - 1):
            # point to seg distance
            segment_dist = self._point_to_segment_distance(point, path[i], path[i+1])
            min_dist = min(min_dist, segment_dist)
        return min_dist

    def _point_to_segment_distance(self, point, seg_start, seg_end):
        # distnace from point to edge maths
        segment = seg_end - seg_start
        to_point = point - seg_start

        # project point to seg
        segment_len_sq = np.dot(segment, segment)
        if segment_len_sq == 0:
            return np.linalg.norm(point - seg_start)

        t = max(0, min(1, np.dot(to_point, segment) / segment_len_sq))

        # closest point on seg to point 
        closest = seg_start + t * segment

        # return distance 
        return np.linalg.norm(point - closest)

    def is_waypoint_accessible(self, waypoint, safety_margin=0.5):
        if self.grid_map is None:
            return True  

        status = self.grid_map.evaluate_footprint(
            waypoint[0], waypoint[1], self.policy.drone_radius + safety_margin
        )

        return status < AirspaceType.RESTRICTED.value

    def attempt_path_recovery(self, off_path_distance):
        # if off path attempt recovery based on strategy
        recovery_strategy = self.cfg.path_recovery_strategy
        recovery_action = self.cfg.path_recovery_action

        if recovery_strategy == PathRecoveryStrategy.RETURN_TO_DEVIATION:
            return self._recover_to_deviation_point(recovery_action)
        elif recovery_strategy == PathRecoveryStrategy.RETURN_TO_CLOSEST_WP:
            return self._recover_to_closest_waypoint(recovery_action)
        else:  # RETURN_TO_NEXT_WP
            return self._recover_to_next_waypoint(recovery_action)

    def _recover_to_deviation_point(self, action):
        # return to point where UAV left path 

        # if deviation point is not known then
        if self.path_deviation_point is None:
            self.path_deviation_point = self.current_path[self.path_index]

        target = self.path_deviation_point

        # if deviation point is unaccesable then
        if not self.is_waypoint_accessible(target):
            return self._recover_to_next_waypoint(action)

        return action, target

    def _recover_to_closest_waypoint(self, action):
        # return to prev or next wp
        # get closest wp
        dist_to_current = np.linalg.norm(self.current_path[self.path_index] - self.pos)
        if self.path_index + 1 < len(self.current_path):
            dist_to_next = np.linalg.norm(self.current_path[self.path_index + 1] - self.pos)
            target = self.current_path[self.path_index + 1] if dist_to_next < dist_to_current else self.current_path[self.path_index]
        else:
            target = self.current_path[self.path_index]

        # if closest wp is unaccesable then
        if not self.is_waypoint_accessible(target):
            return self._recover_to_next_waypoint(action)
        
        return action, target

    def _recover_to_next_waypoint(self, action):
        # recover to next wp on path
        if self.path_index + 1 < len(self.current_path):
            target = self.current_path[self.path_index + 1]
        else:
            target = self.current_path[-1]
        return action, target

    def check_waypoint_blocked(self, waypoint):
        # check if wp is blocked
        if not self.is_waypoint_accessible(waypoint):
            log.warning(f"[{self.env.now:4.1f}] {self.uav_id}: waypoint {waypoint} is now blocked!")
            return True
        return False

    def handle_stuck_uav(self):
        if self.policy is None:
            return False
        
        # Check if hovering for too long
        if self.hover_start_time is None:
            return False

        hover_duration = self.env.now - self.hover_start_time

        # has uav met timeout threashhold
        if hover_duration < self.cfg.hover_timeout:
            return False

        # UAV stuck, attempt recovery
        log.warning(f"[{self.env.now:4.1f}] {self.uav_id}: hovered for {hover_duration:.1f}s, attempting full replan")

        # only attempt replan if theres a valid path
        if self.path_index < len(self.current_path):
            destination = self.current_path[-1] 
            try:
                replanned = self.policy.plan_path(self.pos, destination, self.sm)
                self.current_path = replanned
                self.path_index = 0
                self.hover_start_time = None
                log.info(f"[{self.env.now:4.1f}] {self.uav_id}: full replan successful")
                return True
            except Exception as e:
                log.error(f"[{self.env.now:4.1f}] {self.uav_id}: replan failed: {e}")
                return False

        return False

    def skip_unreachable_waypoint(self):
        # if not at end of path then
        if self.path_index < len(self.current_path):
            current_wp = self.current_path[self.path_index]

            if self.check_waypoint_blocked(current_wp):
                self.skipped_waypoints += 1

                # check how many skiped
                if self.skipped_waypoints > self.cfg.max_waypoint_skips:
                    log.error(f"[{self.env.now:4.1f}] {self.uav_id}: to many skipped waypoints, job failed")
                    return False

                # skip to next wp
                self.path_index += 1
                log.info(f"[{self.env.now:4.1f}] {self.uav_id}: skipped blocked waypoint, now {self.skipped_waypoints} skips")

        return True

    def check_job_timeout(self):
        # check if job is in progresss
        if self.job_start_time is None:
            return True 

        # get time
        job_duration = self.env.now - self.job_start_time

        # has it timed out
        if job_duration > self.cfg.job_timeout:
            log.error(f"[{self.env.now:4.1f}] {self.uav_id}: Job {self.current_job['id']} timeout after {job_duration:.1f}s")
            return False  # timed out

        return True
    
    def execute_flight(self, destination):
        # plan path
        destination_array = np.array(destination, dtype=float)
        self.current_path = self.policy.plan_path(self.pos, destination_array, self.sm)
        self.path_index = 0
        self.job_start_time = self.env.now
        self.skipped_waypoints = 0

        # while not at final wp follow wp
        while self.path_index < len(self.current_path):
            # check if job has timed out
            if not self.check_job_timeout():
                log.error(f"[{self.env.now:4.1f}] {self.uav_id}: ABORTING job due to timeout")
                return

            next_waypoint = self.current_path[self.path_index]

            # check waypoint is valid
            if not self.skip_unreachable_waypoint():
                log.error(f"[{self.env.now:4.1f}] {self.uav_id}: cant reach destination, job failed")
                return 

            # fetch waypoint after potential skip
            if self.path_index >= len(self.current_path):
                break  # at end of path

            next_waypoint = self.current_path[self.path_index]
            self.path_deviation_point = None
            self.recovery_attempts = 0
            self.hover_start_time = None

            # follow edge till wp 
            while np.linalg.norm(next_waypoint - self.pos) > 0.5:
                # check timeout
                if not self.check_job_timeout():
                    return  # Job timed out

                # check hover 
                if self.vel.magnitude() < 0.1:  
                    if self.hover_start_time is None:
                        self.hover_start_time = self.env.now
                else:
                    self.hover_start_time = None
                    self.last_valid_pos = self.pos.copy()

                # check stuck
                if self.handle_stuck_uav():
                    break

                #check off path
                if self.cfg is not None:
                    current_step = int(self.env.now / self.dt)
                    # step mod check to only check every n steps
                    if current_step % self.cfg.off_path_check_interval == 0:
                        off_path_dist = self.calculate_point_to_path_distance(self.pos, self.current_path)

                        # need recovery
                        if off_path_dist > self.cfg.off_path_threshold:
                            recovery_action, recovery_target = self.attempt_path_recovery(off_path_dist)

                            if recovery_action == PathRecoveryAction.RETURN_TO_PATH:
                                next_waypoint = recovery_target 
                            elif recovery_action == PathRecoveryAction.REPLAN:
                                if self.path_index + 1 < len(self.current_path):
                                    replanned = self.policy.plan_path_to_waypoint(
                                        self.pos, self.current_path[self.path_index + 1], self.sm
                                    )
                                    self.current_path[self.path_index:] = replanned[:-1] + self.current_path[self.path_index + 1:]

                            self.recovery_attempts += 1
                            if self.recovery_attempts > self.cfg.max_recovery_attempts:
                                if self.path_index + 1 < len(self.current_path):
                                    next_waypoint = self.current_path[self.path_index + 1]

                # get vel
                new_vel = self.policy.get_velocity(self.uav_id, self.pos, next_waypoint, self.sm)

                # set state
                if new_vel.magnitude() < 0.1:
                    self.state = UAVState.HOVER_WAIT
                else:
                    if np.array_equal(next_waypoint, self.depot):
                        self.state = UAVState.RETURNING
                    else:
                        self.state = UAVState.EN_ROUTE

                self.vel = new_vel

                # update pos
                self.pos += self.vel.as_array() * self.dt
                self.sm.update(self.uav_id, self.pos, self.vel, self.state)

                # collect metrics
                self.metrics.record_path(self.uav_id, self.pos)

                yield self.env.timeout(self.dt)

            # wp reached
            self.path_index += 1

        # arrived at final wp
        self.vel = Velocity(0.0, 0.0)
        if self.sm is not None:
            self.sm.update(self.uav_id, self.pos, self.vel, self.state)
        self.job_start_time = None