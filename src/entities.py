from enum import auto, Enum
import numpy as np
import logging
import json
from .physics import Velocity
from .schemas import JobStatus, WayPointType

from src.physics import GlobalPosition, LocalPosition

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
        self.pos: GlobalPosition = airspace.local_to_world(self.depot_pos)
        self.state = UAVState.IDLE_DEPOT

        # path planning
        self.current_path = []
        self.path_index = 0


        # start process
        # give state to sm
        if self.sm is not None:
            self.sm.update(self.uav_id, self.pos, self.vel, self.state)

        self.action = env.process(self.run())

    def _load_config_vars(self, uav_config_file):
        with open("config/uavs/" + uav_config_file + ".json", 'r') as f:
            config_data = json.load(f)
            for key, value in config_data.items():
                setattr(self, key, value)
    
    def get_global_route(self):

        self.route = self.airspace.get_waypoints(self.pos, self.current_job.target_pos, self.airspace.id, self.current_job.destination_airspace)
        self.route_index = 0

        if self.route is None or len(self.route) < 2:
            raise ValueError("invalid route returned by policy: {}".format(self.route))

    def run(self):
        while True:
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
                case _:
                    raise ValueError(f"Unknown UAV state: {self.state}")   
    
    # wait for job
    def _idle_depot(self):
        # wait for job
        job = yield self.job_queue.get()

        # assign job and change state to takeoff
        self.current_job = job
        self.job_start_time = self.env.now
        
        # get plan
        self.get_global_route()

        if self.route[self.route_index].type != WayPointType.TAKEOFF:
            raise ValueError("first waypoint in route must be TAKEOFF type")

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
                raise ValueError("route index out of bounds for current route")
        
        if self.route[self.route_index].type == WayPointType.HANDOVER_OUT and self.route[self.route_index + 1].type == WayPointType.HANDOVER_IN:
            # initiate handover
            self.state = UAVState.HANDOVER
            return
        
        # execute leg to next waypoint
        self.current_path = self.airspace.plan_leg(self.route[self.route_index], self.route[self.route_index + 1])
        self.path_index = 1

        local_pos = self.airspace.world_to_local(self.pos)
        
        while self.path_index < len(self.current_path):
            yield from self._execute_flight(local_pos, self.current_path[self.path_index])
            self.path_index += 1

        # handle arrival at next waypoint
        if self.route[self.route_index + 1].type == WayPointType.DELIVERY:
            self.state = UAVState.DELIVERING
        elif self.route[self.route_index + 1].type == WayPointType.LANDING:
            self.state = UAVState.LANDING
        elif self.route[self.route_index + 1].type == WayPointType.HANDOVER_IN:
            # invalid state, should have been handled above
            raise ValueError("invalid route: HANDOVER_IN waypoint without following HANDOVER_OUT")
        else:
            self.state = UAVState.EN_ROUTE

        self.route_index += 1
    
    def _delivering(self):

        # check wp is delivery 
        if self.route[self.route_index].type != WayPointType.DELIVERY:
            raise ValueError("current waypoint is not a delivery point for delivering state")
        
        # check at wp
        if np.linalg.norm(np.array([self.pos.x - self.route[self.route_index].position.x, self.pos.y - self.route[self.route_index].position.y])) > 1.0:
            log.info(
                "uav %s delivery mismatch: pos=(%.1f, %.1f) target=(%.1f, %.1f)",
                self.uav_id,
                self.pos.x,
                self.pos.y,
                self.route[self.route_index].position.x,
                self.route[self.route_index].position.y,
            )
            raise ValueError("UAV is not at delivery waypoint position for delivering state")

        # simulate delivery time
        yield self.env.timeout(self.delivery_time_s)
        log.info("uav %s completed delivery for job %s", self.uav_id, self.current_job.id)

        # update job status
        self.current_job.status = JobStatus.COMPLETED
        self.current_job.completion_time = self.env.now

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

        self.current_job = None
        self.state = UAVState.IDLE_DEPOT

    def _handover(self):
        # check current wp is handover out and next wp is handover in
        if self.route[self.route_index].type != WayPointType.HANDOVER_OUT or self.route[self.route_index + 1].type != WayPointType.HANDOVER_IN:
            raise ValueError("invalid route for handover: current waypoint must be HANDOVER_OUT and next waypoint must be HANDOVER_IN")

        out_wp = self.route[self.route_index]
        in_wp = self.route[self.route_index + 1]
        
        # mvp: fixed handover duration
        yield self.env.timeout(1)
        self.airspace.complete_handover(out_wp.airspace_id, in_wp.airspace_id, self.uav_id)

        # update position to next wp
        self.pos = in_wp.position
        print(f"UAV {self.uav_id} completed handover from airspace {out_wp.airspace_id} to airspace {in_wp.airspace_id} at time {self.env.now:.1f}s")

        # update state to en route and increment route index
        self.state = UAVState.EN_ROUTE
        self.route_index += 1

    def _execute_flight(self, start: LocalPosition, target: LocalPosition):
        # simulate flight time based on distance and cruise speed
        distance = np.linalg.norm(np.array([target.x - start.x, target.y - start.y]))
        flight_time = distance / self.cruise_speed_mps
        yield self.env.timeout(flight_time)
        
        # update position and velocity
        self.pos = self.airspace.local_to_world(target)
        log.info("uav %s flew from (%.1f, %.1f) to (%.1f, %.1f)", self.uav_id, start.x, start.y, target.x, target.y)
        self.vel = Velocity(0.0, 0.0)

    def _fly_to_global(self, target: GlobalPosition):
        if np.linalg.norm(np.array([self.pos.x - target.x, self.pos.y - target.y])) <= 1.0:
            return

        local_start = self.airspace.world_to_local(self.pos)
        local_target = self.airspace.world_to_local(target)
        yield from self._execute_flight(local_start, local_target)
        