import json

from src.entities import UAV
from src.environment.map import GridMap
from src.environment.spatial import SpatialManager
import numpy as np

class Airspace:
    def __init__(self, airspace_id, config_file, map_file, fleet_file, env, policy, job_queue, cfg):
        self.id = airspace_id
        self.config = self._load_config(config_file)
        self.origin = np.array(self.config.get('origin', [0.0, 0.0]), dtype=float)

        self.env = env
        self.policy = policy
        self.job_queue = job_queue 
        self.cfg = cfg

        self.map = self._load_map(map_file)
        self.spatial_manager = SpatialManager(10) 
        self.fleet = self._load_fleet(fleet_file, self.map, self.cfg)



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
                            self.job_queue, 
                            depot_x['type'], 
                            self.map, 
                            self.spatial_manager,
                            self.cfg

                            ))
                        break
            # no depot found 
            # log warning 

        # return fleet data
        return fleet

    def world_to_local(self, position):
        return (position - self.origin) / self.map.resolution

    def local_to_world(self, grid_position):
        return grid_position * self.map.resolution + self.origin

    def is_inside(self, position):
        local_pos = self.world_to_local(position)
        return (0 <= local_pos[0] < self.map.grid_width) and (0 <= local_pos[1] < self.map.grid_height)

    def add_uav(self, uav):
        self.uavs.append(uav)   

    def remove_uav(self, uav):
        self.uavs.remove(uav)