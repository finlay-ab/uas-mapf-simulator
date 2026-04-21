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
        self.policy = policy
        self.job_queue = job_queue
        self.uav_config_file = uav_config_file
        self.grid_map = grid_map
        self.sm = sm
        self.cfg = cfg

        self.depot = grid_map.get_depot_position(depot_owner_id)

        self.policy = policy
        self.job_queue = job_queue

        # load uav config vars
        self._load_config_vars(uav_config_file)


        # set uav vars
        self.vel = Velocity(0.0, 0.0)
        self.current_job = None

        # start at depot
        self.depot_pos = self.grid_map.get_depot_position(depot_owner_id)
        self.pos = np.array(self.depot_pos, dtype=float)
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
                case UAVState.RETURNING:
                    yield from self._returning()
                case UAVState.LANDING:
                    yield from self._landing()
                case _:
                    raise ValueError(f"Unknown UAV state: {self.state}")   
    
    def _idle_depot(self):
        # wait for job
        job = yield self.job_queue.get()

        # assign job and change state to takeoff
        self.current_job = job
        self.job_start_time = self.env.now
        self.state = UAVState.TAKEOFF
    
    def _takeoff(self):
        # simulate takeoff time
        yield self.env.timeout(self.takeoff_time_s)

        # plan path to destination
        self.current_path = self.policy.plan_path(self.pos, self.current_job.target_pos, self.grid_map)
        self.path_index = 0

        # change state to en route
        self.state = UAVState.EN_ROUTE

    def _en_route(self):
        # des simulation so yeild time between each weigh point 
        while self.path_index < len(self.current_path):
            next_waypoint = self.current_path[self.path_index]

            distance = np.linalg.norm(next_waypoint - self.pos)
            travel_time = distance / self.cruise_speed_mps
            yield self.env.timeout(travel_time)
            self.pos = next_waypoint
            self.path_index += 1

        # 
        self.state = UAVState.DELIVERING
    
    def _delivering(self):
        # simulate delivery time
        yield self.env.timeout(self.delivery_time_s)

        # change state to returning
        self.state = UAVState.RETURNING

    def _returning(self):
        # des simulation so yeild time between each weigh point 
        while self.path_index < len(self.current_path):
            next_waypoint = self.current_path[self.path_index]

            distance = np.linalg.norm(next_waypoint - self.pos)
            travel_time = distance / self.cruise_speed
            yield self.env.timeout(travel_time)
            self.pos = next_waypoint
            self.path_index += 1

        # 
        self.state = UAVState.LANDING

    def _landing(self):
        # simulate landing time
        yield self.env.timeout(self.takeoff_time_s)

        self.current_job = None
        self.state = UAVState.IDLE_DEPOT

