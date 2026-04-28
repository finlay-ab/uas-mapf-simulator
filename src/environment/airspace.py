import json
import random

from src.entities import UAV
from src.environment.map import GridMap
from src.environment.spatial import SpatialManager
import numpy as np

from src.schemas import Job, JobStatus, Waypoint, WayPointType
from src.physics import GlobalPosition, LocalPosition
import simpy
import logging

log = logging.getLogger("UAS_Sim")  

class Airspace:
    def __init__(self, airspace_id, config_file, map_file, fleet_file, env, policy, cfg, world_manager, metrics):
        self.id = airspace_id
        self.config = self._load_config(config_file)
        self.origin = GlobalPosition(*self.config.get('origin', [0.0, 0.0]))
        self.job_spawn_rate = self.config.get('job_spawn_rate', 1.0)

        self.env = env
        self.policy = policy
        self.job_id = 0
        self.cfg = cfg
        self.world_manager = world_manager
        self.metrics = metrics

        self.map = self._load_map(map_file)
        self.spatial_manager = SpatialManager(10, metrics=metrics) 

        self.job_queues = {}
        # create a job queue for each depot and store in dict in airspaces
        for depot in self.map.depots:
            self.job_queues[depot.id] = simpy.Store(self.env)

        self.depot_resources = {}
        for depot in self.map.depots:
            depot_capacity = self.map.get_depot_operation_capacity(depot.id)
            self.depot_resources[depot.id] = simpy.Resource(self.env, capacity=depot_capacity)

        # set gate capacity
        self.gate_resources = {}
        for gate in self.map.gates:
            self.gate_resources[gate.id] = simpy.Resource(self.env, capacity=gate.capacity)

        log.info(f"Airspace {self.id} initialized with {len(self.map.depots)} depots and job spawn rate {self.job_spawn_rate}.")

        self.fleet = self._load_fleet(fleet_file, self.map, self.cfg)
        
        log.info(f"Airspace {self.id} fleet initialized with {len(self.fleet)} UAVs.")

        # for each depots job queue start a generator that creates jobs for that depot
        for depot in self.map.depots:
            self.env.process(self._job_generator(depot.id))

    def _load_config(self, config_file):
        # load json
        with open(config_file, 'r') as f:
            config_data = json.load(f)
        
        # return config data 
        return config_data

    def _load_map(self, map_file):
        return GridMap(map_file)
        
    def _load_fleet(self, fleet_file, map, cfg):
        # load json
        with open(fleet_file, 'r') as f:
            fleet_data = json.load(f)

        fleet = []

        for depot_cfg in fleet_data['depots']:
            for i in range(depot_cfg['num_uavs']):
                # find matching depot in map
                for depot in self.map.depots:
                    if depot.id == depot_cfg['id']:
                        fleet.append(UAV(
                            self.env, 
                            depot_cfg['id'] + str(len(fleet)), # uav id 
                            depot.id, 
                            self.policy, 
                            self.job_queues[depot.id], 
                            depot_cfg['type'], 
                            self.map, 
                            self.spatial_manager,
                            self.cfg,
                            airspace=self

                            ))
                        break
            # no depot found 
            # log warning 

        # return fleet data
        return fleet

    def _job_generator(self, depot_id):
        log.info(f"Starting job generator for depot {depot_id} in airspace {self.id} with spawn rate {self.job_spawn_rate}.")
        while True:
            # wait poisson interarrival time
            yield self.env.timeout(random.expovariate(self.cfg.lambda_rate/ len(self.map.depots)))

            # default target to same airspace
            destination_airspace = self.id

            # pick different airspace at job swpan rate probabilty
            if random.random() < self.job_spawn_rate:
                other_airspaces = [
                    a for a in self.world_manager.get_all_airspaces().values()
                    if a.id != self.id
                ]
                if other_airspaces:
                    destination_airspace = random.choice(other_airspaces).id

            destination_airspace_obj = self.world_manager.get_airspace(destination_airspace)
            if destination_airspace_obj is None:
                raise ValueError(f"Unknown destination airspace id: {destination_airspace}")

            # random dropoff in destination airspace bounds
            x = random.randrange(destination_airspace_obj.map.grid_width)
            y = random.randrange(destination_airspace_obj.map.grid_height)

            destination = LocalPosition(x, y)
            destination = destination_airspace_obj.local_to_world(destination)

            # create job request and put in queue
            job = Job(
                id=self.id  + "_" + str(self.job_id),
                origin_airspace=self.id,
                origin_depot=depot_id,
                destination_airspace=destination_airspace,  
                target_pos=destination,
                status=JobStatus.PENDING
            )
            self.job_id += 1

            yield self.job_queues[depot_id].put(job)
           
    def local_to_world(self, local_position: LocalPosition) -> GlobalPosition:
        return GlobalPosition(
            local_position.x * self.map.resolution + self.origin.x,
            local_position.y * self.map.resolution + self.origin.y,
        )

    def world_to_local(self, global_position: GlobalPosition) -> LocalPosition:
        return LocalPosition(
            (global_position.x - self.origin.x) / self.map.resolution,
            (global_position.y - self.origin.y) / self.map.resolution,
        )
    
    def is_inside(self, position):
        if isinstance(position, GlobalPosition):
            position = self.world_to_local(position)
        
        if not isinstance(position, LocalPosition):
            raise TypeError("position must be GlobalPosition or LocalPosition")
        
        return self.map.in_bounds(position)

    def get_uav(self, uav_id):
        for uav in self.fleet:
            if uav.uav_id == uav_id:
                return uav
        return None

    def add_uav(self, uav):
        self.fleet.append(uav)   

    def remove_uav(self, uav):
        self.fleet.remove(uav)
    

    def get_waypoints(self, current_position: GlobalPosition, goal_position: GlobalPosition, current_airspace_id, goal_airspace_id) -> list[Waypoint]:
        # check pos are global posittions
        if not isinstance(current_position, GlobalPosition) or not isinstance(goal_position, GlobalPosition):
            raise TypeError("current_position and goal_position must be GlobalPosition")
        
        # return list of waypoints from world_manager to get to goal airspace and then to goal position
        #               #     def get_path(self, start_airspace_id, end_airspace_id, current_pos: GlobalPosition, target_pos: GlobalPosition, target_type: WayPointType) -> list[Waypoint]:

        route = self.world_manager.get_path(current_airspace_id, goal_airspace_id, current_position, goal_position)
        
        # check route is valid
        if route is None or len(route) < 2:
            raise ValueError("no route found from current airspace to goal airspace.")
        
        return route

    def plan_leg(self, current_waypoint: Waypoint, goal_waypoint: Waypoint):
        # check pos are Waypoint
        if not isinstance(current_waypoint, Waypoint) or not isinstance(goal_waypoint, Waypoint):
            raise TypeError("current_waypoint and goal_waypoint must be Waypoint")
        
        # queue at gate for handover
        if current_waypoint.type == WayPointType.HANDOVER_OUT and goal_waypoint.type != WayPointType.HANDOVER_IN:
            # raise error shouldve been handled earlier 
            raise ValueError("invalid handover: current waypoint is HANDOVER_OUT but goal waypoint is not HANDOVER_IN")

        # hand over between airspaces
        if current_waypoint.type == WayPointType.HANDOVER_OUT and goal_waypoint.type == WayPointType.HANDOVER_IN:
           # check gate pair is valid
            if current_waypoint.gate_id != goal_waypoint.gate_id:
                raise ValueError("invalid handover gate pair: current gate {} and goal gate {}".format(current_waypoint.gate_id, goal_waypoint.gate_id))
            
            if current_waypoint.airspace_id != self.id:
                raise ValueError("current waypoint airspace {} does not match current airspace {}".format(current_waypoint.airspace_id, self.id))

            if goal_waypoint.airspace_id == self.id:
                raise ValueError("invalid handover: HANDOVER_IN waypoint must belong to a different airspace")
            
            raise ValueError("invalid handover: should be handled by HANDOVER state")

        # en route
        # if current_waypoint.type == WayPointType.EN_ROUTE and goal_waypoint.type != WayPointType.HANDOVER_IN and goal_waypoint.type != WayPointType.HANDOVER_OUT:
        return self.plan_path(current_waypoint.position, goal_waypoint.position)
        
        # invalid waypoint pair
        raise ValueError("invalid waypoint pair: current_waypoint type {} and goal_waypoint type {}".format(current_waypoint.type, goal_waypoint.type))
        
         

    def plan_path(self, current_position: GlobalPosition, goal_position: GlobalPosition):
        # check pos are global posittions 
        if not isinstance(current_position, GlobalPosition) or not isinstance(goal_position, GlobalPosition):
            raise TypeError("current_position and goal_position must be GlobalPosition")

        # check pos are inside airspace
        if not self.is_inside(current_position) or not self.is_inside(goal_position):
            raise ValueError("current position or goal position is outside of airspace bounds.")
        
        # convert to local positions
        local_start = self.world_to_local(current_position)
        local_goal = self.world_to_local(goal_position)

        # plan path in current airspace
        path = self.policy.plan_path(local_start, local_goal, self.spatial_manager)

        if path is None or len(path) < 2:
            raise ValueError("no path found from current position to goal position within airspace.")
        
        return path
        
    def request_gate(self, gate_id):
        if gate_id not in self.gate_resources:
            raise ValueError(f"Unknown gate id: {gate_id}")
        return self.gate_resources[gate_id].request()

    def release_gate(self, gate_id, req):
        if gate_id not in self.gate_resources:
            raise ValueError(f"Unknown gate id: {gate_id}")
        self.gate_resources[gate_id].release(req)

    def get_gate_global_position(self, gate_id):
        gate = self.map.get_gate(gate_id)
        if gate is None:
            raise ValueError(f"unknown gate id: {gate_id}")
        gate_local = self.map.grid_to_local(gate.position)
        return self.local_to_world(gate_local)

    def get_gate_queue_global_positions(self, gate_id):
        queue_positions = self.map.get_gate_queue_positions(gate_id)
        queue_global_positions = []
        for queue_pos in queue_positions:
            queue_local = self.map.grid_to_local(queue_pos)
            queue_global_positions.append(self.local_to_world(queue_local))
        return queue_global_positions

    def get_gate_queue_global_position_for_rank(self, gate_id, queue_rank):
        queue_positions = self.get_gate_queue_global_positions(gate_id)
        if not queue_positions:
            return self.get_gate_global_position(gate_id)

        # mvp behavior: overflow queue ranks hold at last configured slot.
        queue_idx = min(max(queue_rank, 0), len(queue_positions) - 1)
        return queue_positions[queue_idx]

    def request_depot_slot(self, depot_id):
        if depot_id not in self.depot_resources:
            raise ValueError(f"Unknown depot id: {depot_id}")
        return self.depot_resources[depot_id].request()

    def release_depot_slot(self, depot_id, req):
        if depot_id not in self.depot_resources:
            raise ValueError(f"Unknown depot id: {depot_id}")
        self.depot_resources[depot_id].release(req)

    def get_depot_global_position(self, depot_id):
        depot_local = self.map.get_depot_position(depot_id)
        return self.local_to_world(depot_local)

    def get_depot_queue_global_positions(self, depot_id):
        queue_positions = self.map.get_depot_queue_positions(depot_id)
        queue_global_positions = []
        for queue_pos in queue_positions:
            queue_local = self.map.grid_to_local(queue_pos)
            queue_global_positions.append(self.local_to_world(queue_local))
        return queue_global_positions

    def get_depot_queue_global_position_for_rank(self, depot_id, queue_rank):
        queue_positions = self.get_depot_queue_global_positions(depot_id)
        if not queue_positions:
            return self.get_depot_global_position(depot_id)

        # mvp behavior: overflow queue ranks hold at last configured slot.
        queue_idx = min(max(queue_rank, 0), len(queue_positions) - 1)
        return queue_positions[queue_idx]

    def complete_handover(self, from_airspace_id, to_airspace_id, uav_id):
        self.world_manager.complete_handover(from_airspace_id, to_airspace_id, uav_id)