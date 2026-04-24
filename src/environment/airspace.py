import json
import random

from src.entities import UAV
from src.environment.map import GridMap
from src.environment.spatial import SpatialManager
import numpy as np

from src.schemas import Job, JobStatus
from src.physics import GlobalPosition, LocalPosition
import simpy
import logging

log = logging.getLogger("UAS_Sim")  

class Airspace:
    def __init__(self, airspace_id, config_file, map_file, fleet_file, env, policy,cfg, world_manager):
        self.id = airspace_id
        self.config = self._load_config(config_file)
        self.origin = GlobalPosition(*self.config.get('origin', [0.0, 0.0]))
        self.job_spawn_rate = self.config.get('job_spawn_rate', 1.0)

        self.env = env
        self.policy = policy
        self.job_id = 0
        self.cfg = cfg
        self.world_manager = world_manager

        self.map = self._load_map(map_file)
        self.spatial_manager = SpatialManager(10) 

        self.job_queues = {}
        # create a job queue for each depot and store in dict in airspaces
        for depot in self.map.depots:
            self.job_queues[depot.id] = simpy.Store(self.env)

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

        for depot_x in fleet_data['depots']:
            for x in range(depot_x  ['num_uavs']):
                # find depot in map that matches depot_id
                for depot_y in self.map.depots:
                    if depot_y.id == depot_x['id']:
                        fleet.append(UAV(
                            self.env, 
                            depot_x['id'] + str(len(fleet)), # uav id 
                            depot_y.id, 
                            self.policy, 
                            self.job_queues[depot_y.id], 
                            depot_x['type'], 
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

            # pick airspace  
            destination_airspace = self.world_manager.get_airspace(self.id)

            # get random dropoff location in same airspace for now just random x,y in map bounds
            # random self.map.grid_height, self.map.grid_width
            x = random.randrange(self.map.grid_width)
            y = random.randrange(self.map.grid_height)  

            # make np array and convert to world coordinates
            destination = np.array([x, y], dtype=float)
            destination = self.local_to_world(destination)

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

    def add_uav(self, uav):
        self.uavs.append(uav)   

    def remove_uav(self, uav):
        self.uavs.remove(uav)