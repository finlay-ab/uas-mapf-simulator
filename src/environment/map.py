import json

import numpy as np
import logging
from enum import Enum, auto 

# init logging
log = logging.getLogger("UAS_Sim")

# in order of severity 
class AirspaceType(Enum):
    OPEN = 0
    RESTRICTED = auto()
    PROHIBITED = auto()
    OBSTACLE = auto()

class Depot:
    def __init__(self, depot_id, x, y):
        self.id = depot_id
        self.x = x
        self.y = y

class GridMap:
    def __init__(self, map_file):
        # load config
        with open(map_file, 'r') as f:
            map_data = json.load(f) 

        # load map data 
        self.resolution = map_data.get('resolution', 1.0)
        self.grid_width = int(np.ceil(map_data.get('width', 100) / self.resolution))
        self.grid_height = int(np.ceil(map_data.get('height', 100) / self.resolution))
        self.potential_field = map_data.get('potential_field', True)
        self.potential_strength = map_data.get('potential_strength', 10.0)

        # initilize grids
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=int)
        
        # initilize weight map
        self.weighted_grid = np.ones((self.grid_width, self.grid_height), dtype=float)

        # store depots
        self.depots = []
        self.depot_counter = 0

        # load obstacles and depots from config
        self._load_from_config(map_data)

        if self.potential_field:
            self.apply_potential_field(self.potential_strength)

        if 1 == 1:
            # code to dispaly the map for debugging/ graphs
            pass
            

    # convert floating co-ordinates into grid co ordinates
    def get_grid_indices(self, x_min, x_max, y_min, y_max):
        gx_min = max(0, int(x_min / self.resolution))
        gx_max = min(self.grid_width, int(np.ceil(x_max / self.resolution)))
        gy_min = max(0, int(y_min / self.resolution))
        gy_max = min(self.grid_height, int(np.ceil(y_max / self.resolution)))

        return gx_min, gx_max, gy_min, gy_max

    def world_to_grid_point(self, x, y, clamp=True):
        gx = int(float(x) / self.resolution)
        gy = int(float(y) / self.resolution)

        if clamp:
            gx = min(max(gx, 0), self.grid_width - 1)
            gy = min(max(gy, 0), self.grid_height - 1)

        return gx, gy

    def grid_to_world_point(self, gx, gy, center=True):
        gx = int(gx)
        gy = int(gy)
        offset = 0.5 if center else 0.0
        return (
            (gx + offset) * self.resolution,
            (gy + offset) * self.resolution,
        )

    # makes rect obstacle in grid
    def add_obstacle(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.get_grid_indices(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.OBSTACLE.value
    
    def add_restricted_area(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.get_grid_indices(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.RESTRICTED.value

    # adds a depot location
    def add_depot(self,depot_id, x, y):
        new_depot = Depot(depot_id, x, y)
        self.depots.append(new_depot)
        
    def apply_potential_field(self, strength):
        # very inefficient but only runs once
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if self.grid[x, y] == AirspaceType.OBSTACLE.value:
                    for nx in range(max(0, x-5), min(self.grid_width, x+6)):
                        for ny in range(max(0, y-5), min(self.grid_height, y+6)):
                            if self.grid[nx, ny] != AirspaceType.OBSTACLE.value:
                                dist = np.sqrt((nx-x)**2 + (ny-y)**2)
                                if dist > 0 and dist <= 5:
                                    self.weighted_grid[nx, ny] += (strength / dist)

    def load_from_file(self, file):
        try:
            with open(file, 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) != 5:
                        continue
                    type_str, x_min, x_max, y_min, y_max = parts
                    x_min, x_max, y_min, y_max = map(float, [x_min, x_max, y_min, y_max])
                    if type_str == "OBSTACLE":
                        self.add_obstacle(x_min, x_max, y_min, y_max)
                    elif type_str == "RESTRICTED":
                        self.add_restricted_area(x_min, x_max, y_min, y_max)
                    elif type_str == "DEPOT":
                        self.add_depot(x_min, x_max, y_min, y_max)
        except FileNotFoundError:
            log.warning(f"Map file {file} not found. Proceeding with empty map.")

    def evaluate_footprint(self, gx, gy, radius):
        # Convert physical coordinates to grid indices
        cx = int(gx / self.resolution)
        cy = int(gy / self.resolution)
        grid_rad = int(np.ceil(radius / self.resolution))
        worst = AirspaceType.OPEN.value
        
        for nx in range(cx - grid_rad, cx + grid_rad + 1):
            for ny in range(cy - grid_rad, cy + grid_rad + 1):
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                    worst = max(worst, self.grid[nx, ny])
                else:
                    worst = max(worst, AirspaceType.PROHIBITED.value) 
        return worst

    def is_traversable(self, gx, gy, drone_radius):
        if not (0 <= gx < self.grid_width and 0 <= gy < self.grid_height):
            return False

        x, y = self.grid_to_world_point(gx, gy, center=True)
        status = self.evaluate_footprint(x, y, drone_radius)
        return status < AirspaceType.RESTRICTED.value

    def get_neighbors(self, gx, gy, drone_radius, connectivity=4):
        cardinal = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in cardinal:
            nxt_gx, nxt_gy = gx + dx, gy + dy
            if self.is_traversable(nxt_gx, nxt_gy, drone_radius):
                yield (nxt_gx, nxt_gy)

        if connectivity == 8:
            diagonal = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dx, dy in diagonal:
                nxt_gx, nxt_gy = gx + dx, gy + dy
                if not self.is_traversable(nxt_gx, nxt_gy, drone_radius):
                    continue

                side_a = self.is_traversable(gx + dx, gy, drone_radius)
                side_b = self.is_traversable(gx, gy + dy, drone_radius)
                if side_a and side_b:
                    yield (nxt_gx, nxt_gy)

    def _load_from_config(self, map_data):
        for obstacle in map_data.get('obstacles', []):
            self.add_obstacle(obstacle['x_min'], obstacle['x_max'], obstacle['y_min'], obstacle['y_max'])
        
        for restricted in map_data.get('restricted_areas', []):
            self.add_restricted_area(restricted['x_min'], restricted['x_max'], restricted['y_min'], restricted['y_max'])
        
        for depot in map_data.get('depots', []):
            self.add_depot(depot['id'], depot['x'], depot['y'])

    # safely fetch a depot location for drones to start at
    def get_depot_spawn(self, index=0):
        if len(self.depots) == 0:
            return 0.0, 0.0
        
        # use modulo so if you have 10 drones and 2 depots, they alternate safely
        depot = self.depots[index % len(self.depots)]
        return depot.x, depot.y
    
    def get_depot_position(self, depot_id):
        for depot in self.depots:
            if depot.id == depot_id:
                # return np arr
                return np.array([depot.x, depot.y], dtype=float)
            
        log.warning(f"Depot with id {depot_id} not found. Returning (0,0).")
        return 0.0, 0.0
            