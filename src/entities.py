from enum import auto, Enum
import numpy as np
import logging
import json
from .physics import Velocity
from .schemas import JobStatus, WayPointType, UAV_SEGMENT
from .helpers import get_xy, to_array

from src.physics import GlobalPosition, LocalPosition, Velocity

# get log
log = logging.getLogger("UAS_Sim")

# enum to track state for metrics
class UAVState(Enum):
    IDLE_DEPOT = auto()
    TAKEOFF = auto()
    LANDING = auto()
    EN_ROUTE = auto()
    HOVER_WAIT = auto()
    DELIVERING = auto()
    RETURNING = auto()
    EMERGENCY = auto()
    BLOCKED = auto()
    HANDOVER = auto()
    

# UAV entity which stores state and jobs
class UAV:
    def __init__(self, env, uav_id, depot_owner_id, policy, job_queue, uav_config_file, grid_map, sm, cfg, airspace):
        # set input vars
        self.env = env
        self.uav_id = uav_id
        self.depot_owner_id = depot_owner_id
        self.policy = policy
        self.job_queue = job_queue
        self.uav_config_file = uav_config_file
        self.grid_map = grid_map
        self.sm = sm
        self.cfg = cfg
        self.airspace = airspace
        self.allow_predicted_collisions = getattr(cfg, "allow_predicted_collisions", True)

        self.depot = grid_map.get_depot_position(depot_owner_id)

        self.policy = policy
        self.job_queue = job_queue

        # load uav config vars
        self._load_config_vars(uav_config_file)


        # set uav vars
        self.vel = Velocity(0.0, 0.0)
        self.current_job = None

        # start at depot
        self.depot_pos: GlobalPosition = airspace.local_to_world(self.grid_map.get_depot_position(depot_owner_id))
        self.pos: GlobalPosition = self.depot_pos
        self.state = UAVState.IDLE_DEPOT

        # path planning
        self.current_path = []
        self.path_index = 0

        # replan vars 
        self.replan_attempts = 0
        self.emergency_attempts = 0
        self._last_partial_distance = float("inf")


        # start process
        # give state to sm
        if self.sm is not None:
            self.sm.update_position_snapshot(self.uav_id, self.pos, self.state)

        self.action = env.process(self.run())

    def _load_config_vars(self, uav_config_file):
        with open("config/uavs/" + uav_config_file + ".json", 'r') as f:
            config_data = json.load(f)
            for key, value in config_data.items():
                setattr(self, key, value)
    
    # returns True if a valid multi-airspace route was found
    def get_global_route(self):
        self.route = self.airspace.get_waypoints(self.pos, self.current_job.target_pos, self.airspace.id, self.current_job.destination_airspace)
        self.route_index = 0
        if self.route is None or len(self.route) < 2:
            return False
        return True

    def run(self):
        last_state_synced = None
        while True:
            # update spartial manager
            if self.sm is not None and self.state != last_state_synced:
                self.sm.update_state(self.uav_id, self.state)
                last_state_synced = self.state

            match self.state:
                case UAVState.IDLE_DEPOT:
                    yield from self._idle_depot()
                case UAVState.TAKEOFF:
                    yield from self._takeoff()
                case UAVState.EN_ROUTE:
                    yield from self._en_route()
                case UAVState.HOVER_WAIT:
                    yield from self._hover_wait()
                case UAVState.DELIVERING:
                    yield from self._delivering()
                case UAVState.LANDING:
                    yield from self._landing()
                case UAVState.HANDOVER:
                    yield from self._handover()
                case UAVState.EMERGENCY:
                    yield from self._emergency()
                case _:
                    raise ValueError(f"Unknown UAV state: {self.state}")   
    
    # wait for job
    def _idle_depot(self):
        # wait for job
        job = yield self.job_queue.get()

        # assign job 
        self.current_job = job
        self.job_start_time = self.env.now
        self.current_job.job_start_time = self.env.now

        # if no route fail job
        if not self.get_global_route():
            log.warning("uav %s could not plan global route for job %s, failing", self.uav_id, job.id)
            self.airspace.metrics.record_job_failed_at_depot(
                depot_id=self.depot_owner_id,
                job_id=job.id,
                reason="no_global_route",
                failed_time=self.env.now
            )
            self.current_job = None
            return

        # update job status
        self.airspace.metrics.record_job_in_progress_at_depot(
            job_id=self.current_job.id,
            started_time=self.env.now
        )
        
        log.info("uav %s assigned job %s (%s -> %s)", self.uav_id, job.id, job.origin_airspace, job.destination_airspace)

        if self.route[self.route_index].type != WayPointType.TAKEOFF:
            log.error("uav %s first waypoint not TAKEOFF (got %s)", self.uav_id, self.route[self.route_index].type)
            self.airspace.metrics.record_job_failed_at_depot(
                depot_id=self.depot_owner_id,
                job_id=job.id,
                reason="invalid_route",
                failed_time=self.env.now
            )
            self.current_job = None
            return

        self.replan_attempts = 0
        self.state = UAVState.TAKEOFF

    # reserve a depot slot before departure
    def _takeoff(self):
        depot_req = self.airspace.request_depot_slot(self.depot_owner_id)

        while not depot_req.triggered:
            depot_resource = self.airspace.depot_resources[self.depot_owner_id]
            queue_rank = 0

            if depot_req in depot_resource.queue:
                queue_rank = depot_resource.queue.index(depot_req)

            queue_hold_position = self.airspace.get_depot_queue_global_position_for_rank(self.depot_owner_id, queue_rank)
            yield from self._fly_to_global(queue_hold_position)
            yield self.env.timeout(self.cfg.depot_check_interval)

        yield depot_req
        yield from self._fly_to_global(self.depot_pos)

        # simulate takeoff time
        yield self.env.timeout(self.takeoff_time_s)
        self.airspace.release_depot_slot(self.depot_owner_id, depot_req)

        # change state to en route
        self.state = UAVState.EN_ROUTE

    # handle en route
    def _en_route(self):

        # check if enough routes left in current plan
        if self.route_index >= len(self.route) - 1:
            # if landing or out of waypoints
            if self.route[self.route_index].type == WayPointType.LANDING:
                self.state = UAVState.LANDING
                return
            else:
                log.error("uav %s route index %d out of bounds %d, going EMERGENCY", self.uav_id, self.route_index, len(self.route))
                self.state = UAVState.EMERGENCY
                return

        next_wp = self.route[self.route_index + 1]

        if self.route[self.route_index].type == WayPointType.HANDOVER_OUT and next_wp.type == WayPointType.HANDOVER_IN:
            # initiate handover
            self.state = UAVState.HANDOVER
            return

        # try to plan to next wp
        try:
            self.current_path = self.airspace.plan_path(self.pos, next_wp.position, uav_id=self.uav_id)
        except Exception as exc:
            log.warning("uav %s plan_path failed: %s", self.uav_id, exc)
            self.current_path = None

        # if no path then hover and wait for next replan
        if self.current_path is None or len(self.current_path) < 2:
            self.replan_attempts += 1
            if self.replan_attempts >= self.cfg.max_recovery_attempts:
                log.warning("uav %s replan attempts exhausted, going EMERGENCY", self.uav_id)
                self.state = UAVState.EMERGENCY
            else:
                self.state = UAVState.HOVER_WAIT
            return

        # got a path, reset retry counter
        self.replan_attempts = 0
        self.path_index = 1

        while self.path_index < len(self.current_path):
            local_pos = self.airspace.world_to_local(self.pos)
            flew = yield from self._execute_flight(local_pos, self.current_path[self.path_index])
            if not flew:
                log.warning("uav %s flight blocked, hovering", self.uav_id)
                self.state = UAVState.HOVER_WAIT
                return
            self.path_index += 1

        # check if wp is reached or if replan is needed
        current_xy = to_array(self.pos)
        target_xy = to_array(next_wp.position)
        distance_to_wp = float(np.linalg.norm(target_xy - current_xy))

        if distance_to_wp > self.cfg.waypoint_arrival_threshold:
            # prevent looping/ crashing by ensuring progress is made
            if distance_to_wp >= self._last_partial_distance - 0.5:
                self.replan_attempts += 1
                if self.replan_attempts >= self.cfg.max_recovery_attempts:
                    log.warning("uav %s partial path not making progress, going EMERGENCY", self.uav_id)
                    self._last_partial_distance = float("inf")
                    self.state = UAVState.EMERGENCY
                    return
                self._last_partial_distance = distance_to_wp
                self.state = UAVState.HOVER_WAIT
                return

            # making progress, continue replanning toward the waypoint
            self._last_partial_distance = distance_to_wp
            self.state = UAVState.EN_ROUTE
            return
        
        self._last_partial_distance = float("inf")

        # reached the waypoint
        if next_wp.type == WayPointType.DELIVERY:
            self.state = UAVState.DELIVERING
        elif next_wp.type == WayPointType.LANDING:
            self.state = UAVState.LANDING
        elif next_wp.type == WayPointType.HANDOVER_IN:
            # invalid state, should have been handled above
            log.error("uav %s unexpected HANDOVER_IN waypoint, going EMERGENCY", self.uav_id)
            self.state = UAVState.EMERGENCY
            return
        else:
            self.state = UAVState.EN_ROUTE

        self.route_index += 1

    # hold position
    def _hover_wait(self):
        yield self.env.timeout(self.cfg.hover_timeout)
        self.state = UAVState.EN_ROUTE

    # abort job
    def _emergency(self):
        # if grounded then wait indefinitely
        if getattr(self, "grounded", False):
            yield self.env.timeout(self.cfg.sim_time)
            return

        # only fail the job once even if emergency repeats
        if self.current_job is not None:
            log.warning("uav %s aborting job %s, attempting to return home", self.uav_id, self.current_job.id)
            self.airspace.metrics.record_job_failed_at_depot(
                depot_id=self.depot_owner_id,
                job_id=self.current_job.id,
                reason="path_unreachable",
                failed_time=self.env.now
            )
            self.current_job = None
            self.emergency_attempts = 0

        # try to return to home airspace
        if not self.airspace.is_inside(self.depot_pos):
            self.emergency_attempts += 1
            if self.emergency_attempts >= self.cfg.max_recovery_attempts:
                log.warning("uav %s stranded in foreign airspace %s, grounding in place", self.uav_id, self.airspace.id)
                yield from self._ground_in_place()
                return
            log.warning("uav %s stranded in foreign airspace %s, hovering", self.uav_id, self.airspace.id)
            yield self.env.timeout(self.cfg.hover_timeout)
            return

        # check if above depot and can land safely
        current_xy = to_array(self.pos)
        depot_xy = to_array(self.depot_pos)
        distance_to_depot = float(np.linalg.norm(depot_xy - current_xy))
        if distance_to_depot <= self.cfg.waypoint_arrival_threshold:
            self.replan_attempts = 0
            self.emergency_attempts = 0
            self.state = UAVState.LANDING
            return

        # try to path home
        try:
            home_path = self.airspace.plan_path(self.pos, self.depot_pos, uav_id=self.uav_id)
        except Exception as exc:
            log.warning("uav %s emergency plan home failed: %s", self.uav_id, exc)
            home_path = None

        if home_path is None or len(home_path) < 2:
            self.emergency_attempts += 1
            if self.emergency_attempts >= self.cfg.max_recovery_attempts:
                log.warning("uav %s cannot find path home after %d attempts, grounding in place", self.uav_id, self.emergency_attempts)
                yield from self._ground_in_place()
                return
            log.warning("uav %s cannot find path home, hovering (attempt %d)", self.uav_id, self.emergency_attempts)
            yield self.env.timeout(self.cfg.hover_timeout)
            return

        # fly home along the recovery path
        self.current_path = home_path
        self.path_index = 1
        while self.path_index < len(self.current_path):
            local_pos = self.airspace.world_to_local(self.pos)
            flew = yield from self._execute_flight(local_pos, self.current_path[self.path_index])
            if not flew:
                log.warning("uav %s emergency flight blocked, hovering", self.uav_id)
                yield self.env.timeout(self.cfg.hover_timeout)
                return
            self.path_index += 1

        # at home
        log.info("uav %s reached home depot in emergency, landing", self.uav_id)
        self.replan_attempts = 0
        self.emergency_attempts = 0
        self.state = UAVState.LANDING

    # UAV emergancy landing!
    def _ground_in_place(self):
        if self.sm is not None:
            # remove from active segments
            if self.uav_id in self.sm.active_segments:
                self.sm.deregister_segment(self.uav_id, self.pos)
            # update state in sm
            self.sm.update_state(self.uav_id, UAVState.IDLE_DEPOT)
        # make sure dispatch doesnt assign new jobs
        self.grounded = True
        log.warning("uav %s grounded; will not take further jobs", self.uav_id)
        yield self.env.timeout(self.cfg.sim_time)

    def _delivering(self):

        # check wp is delivery 
        if self.route[self.route_index].type != WayPointType.DELIVERY:
            # routing error
            log.error("uav %s entered DELIVERING but waypoint is %s, going EMERGENCY", self.uav_id, self.route[self.route_index].type)
            self.state = UAVState.EMERGENCY
            return
        
        # check at wp. if we are not actually here, drop back to en route so the planner can try again
        current_xy = to_array(self.pos)
        target_xy = to_array(self.route[self.route_index].position)
        distance_to_target = np.linalg.norm(target_xy - current_xy)

        if distance_to_target > self.cfg.waypoint_arrival_threshold:
            current_x, current_y = get_xy(self.pos)
            target_x, target_y = get_xy(self.route[self.route_index].position)
            log.warning(
                "uav %s delivery mismatch: pos=(%.1f, %.1f) target=(%.1f, %.1f), going back to EN_ROUTE",
                self.uav_id,
                current_x,
                current_y,
                target_x,
                target_y,
            )
            # roll back to the previous waypoint so en_route will replan to the delivery point
            self.route_index -= 1
            self.state = UAVState.EN_ROUTE
            return

        # simulate delivery time
        yield self.env.timeout(self.delivery_time_s)
        
        # update job status
        self.current_job.status = JobStatus.COMPLETED
        self.current_job.job_completion_time = self.env.now
        
        # calculate delivery time
        delivery_time = self.env.now - self.current_job.job_start_time
        
        # record delivery completion in metrics
        self.airspace.metrics.record_delivery_complete_at_depot(
            depot_id=self.depot_owner_id,
            job_id=self.current_job.id,
            delivery_time=delivery_time,
            completed_time=self.env.now
        )
        
        log.info("uav %s completed delivery for job %s (%.2fs)", self.uav_id, self.current_job.id, delivery_time)

        # update current state
        self.state = UAVState.EN_ROUTE

    def _landing(self):
        depot_req = self.airspace.request_depot_slot(self.depot_owner_id)

        while not depot_req.triggered:
            depot_resource = self.airspace.depot_resources[self.depot_owner_id]
            queue_rank = 0

            if depot_req in depot_resource.queue:
                queue_rank = depot_resource.queue.index(depot_req)

            queue_hold_position = self.airspace.get_depot_queue_global_position_for_rank(self.depot_owner_id, queue_rank)
            yield from self._fly_to_global(queue_hold_position)
            yield self.env.timeout(self.cfg.depot_check_interval)

        yield depot_req
        yield from self._fly_to_global(self.depot_pos)

        # simulate landing time
        yield self.env.timeout(self.takeoff_time_s)
        self.airspace.release_depot_slot(self.depot_owner_id, depot_req)

        # update pos snapshot at landing
        if self.sm is not None:
            self.sm.update_position_snapshot(self.uav_id, self.pos, UAVState.IDLE_DEPOT)

        self.current_job = None
        self.state = UAVState.IDLE_DEPOT

    def _handover(self):
        # check current wp is handover out and next wp is handover in
        if self.route[self.route_index].type != WayPointType.HANDOVER_OUT or self.route[self.route_index + 1].type != WayPointType.HANDOVER_IN:
            log.error("uav %s entered HANDOVER with invalid waypoints, going EMERGENCY", self.uav_id)
            self.state = UAVState.EMERGENCY
            return

        out_wp = self.route[self.route_index]
        in_wp = self.route[self.route_index + 1]
        
        # mvp: fixed handover duration
        yield self.env.timeout(1)
        
        # deregister from source airspace spatial manager before handover
        if self.sm is not None:
            self.sm.deregister_segment(self.uav_id, out_wp.position)
        
        # get destination airspace object
        destination_airspace = self.airspace.world_manager.get_airspace(in_wp.airspace_id)
        if destination_airspace is None:
            log.error("uav %s handover destination airspace %s not found, going EMERGENCY", self.uav_id, in_wp.airspace_id)
            self.state = UAVState.EMERGENCY
            return
        
        self.airspace.complete_handover(out_wp.airspace_id, in_wp.airspace_id, self.uav_id)

        # update position to next wp
        self.pos = in_wp.position
        
        # switch spatial manager reference to destination airspace
        self.airspace = destination_airspace
        self.sm = destination_airspace.spatial_manager
        
        # register with destination airspace's spatial manager
        if self.sm is not None:
            self.sm.update_position_snapshot(self.uav_id, self.pos, UAVState.EN_ROUTE)
        
        log.info(f"UAV {self.uav_id} completed handover from airspace {out_wp.airspace_id} to airspace {in_wp.airspace_id} at time {self.env.now:.1f}s")

        # update state to en route and increment route index
        self.state = UAVState.EN_ROUTE
        self.route_index += 1

    def _execute_flight(self, start: LocalPosition, target: LocalPosition):
        # convert local positions to global for segment registration
        start_global = self.airspace.local_to_world(start)
        target_global = self.airspace.local_to_world(target)
        
        # simulate flight time based on distance and cruise speed
        start_xy = to_array(start)
        target_xy = to_array(target)
        distance = np.linalg.norm(target_xy - start_xy)
        flight_time = distance / self.cruise_speed_mps
        
        # create velocity vector from start to target
        direction = target_xy - start_xy
        if distance > 0:
            direction = direction / distance
            vel_magnitude = self.cruise_speed_mps
        else:
            direction = np.array([0.0, 0.0])
            vel_magnitude = 0.0
        velocity = Velocity(direction[0] * vel_magnitude, direction[1] * vel_magnitude)
        
        # build UAV_SEGMENT for this flight leg
        segment = UAV_SEGMENT(
            start_position=start_global,
            end_position=target_global,
            start_time=self.env.now,
            end_time=self.env.now + flight_time,
            velocity=velocity,
            radius=self.body_radius_m
        )
        
        # check safety before committing to segment
        if self.sm is not None:
            safe = self.sm.check_segment_safety(self.uav_id, segment)
            if not safe:
                log.warning("uav %s segment collision predicted", self.uav_id)
                if not self.allow_predicted_collisions:
                    return False

            # always register the segment so baseline planners can run through unsafe motion
            self.sm.register_segment(self.uav_id, segment)
        
        # yield for flight duration 
        yield self.env.timeout(flight_time)
        
        # on arrival deregister segment 
        self.pos = target_global
        if self.sm is not None:
            self.sm.deregister_segment(self.uav_id, self.pos)
        
        start_x, start_y = get_xy(start)
        target_x, target_y = get_xy(target)
        log.info("uav %s flew from (%.1f, %.1f) to (%.1f, %.1f)", self.uav_id, start_x, start_y, target_x, target_y)
        self.vel = Velocity(0.0, 0.0)
        return True

    def _fly_to_global(self, target: GlobalPosition):
        current_xy = to_array(self.pos)
        target_xy = to_array(target)
        distance = np.linalg.norm(target_xy - current_xy)

        if distance <= 1.0:
            return

        local_start = self.airspace.world_to_local(self.pos)
        local_target = self.airspace.world_to_local(target)
        yield from self._execute_flight(local_start, local_target)