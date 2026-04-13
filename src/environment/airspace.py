from src.environment.map import GridMap
import numpy as np

class Airspace:
    def __init__(self, config, spatial_manager, uavs=None):
        self.id = config['id']
        self.map = GridMap(config['map_width'], config['map_height'], config.get('potential_field', False), config.get('potential_strength', 10.0), config.get('file', None))
        self.spatial_manager = spatial_manager
        self.origin = np.array(config.get('origin', [0.0, 0.0]), dtype=float)

        if uavs is None:
            self.uavs = []
        else:
            self.uavs = uavs

    def world_to_local(self, position):
        return (position - self.origin) / self.map.resolution

    def local_to_world(self, grid_position):
        return grid_position * self.map.resolution + self.origin

    def is_inside(self, position):
        local_pos = self.world_to_local(position)
        return (0 <= local_pos[0] < self.map.size[0]) and (0 <= local_pos[1] < self.map.size[1])

    def add_uav(self, uav):
        self.uavs.append(uav)   

    def remove_uav(self, uav):
        self.uavs.remove(uav)

        