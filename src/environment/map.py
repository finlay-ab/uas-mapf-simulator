from collections import deque
import json

import numpy as np
import logging
from enum import Enum, auto 

from src.physics import LocalPosition, GridPosition
from src.schemas import AirspaceType, Depot


# init logging
log = logging.getLogger("UAS_Sim")



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

    # convert floating co-ordinates into grid co ordinates
    def local_rect_to_grid(self, x_min, x_max, y_min, y_max):
        gx_min = max(0, int(x_min / self.resolution))
        gx_max = min(self.grid_width, int(np.ceil(x_max / self.resolution)))
        gy_min = max(0, int(y_min / self.resolution))
        gy_max = min(self.grid_height, int(np.ceil(y_max / self.resolution)))

        return gx_min, gx_max, gy_min, gy_max





    # makes rect obstacle in grid
    def add_obstacle(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.local_rect_to_grid(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.OBSTACLE.value
    
    def add_restricted_area(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.local_rect_to_grid(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.RESTRICTED.value

    # adds a depot location
    def add_depot(self,depot_id, position):
        if isinstance(position, LocalPosition):
            position = self.local_to_grid(position)
        
        if not isinstance(position, GridPosition):
            raise TypeError("position must be LocalPosition or GridPosition")
        
        new_depot = Depot(depot_id, position)
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

    def _load_from_config(self, map_data):
        for obstacle in map_data.get('obstacles', []):
            self.add_obstacle(obstacle['x_min'], obstacle['x_max'], obstacle['y_min'], obstacle['y_max'])
        
        for restricted in map_data.get('restricted_areas', []):
            self.add_restricted_area(restricted['x_min'], restricted['x_max'], restricted['y_min'], restricted['y_max'])
        
        for depot in map_data.get('depots', []):
            position = LocalPosition(depot['x'], depot['y'])
            self.add_depot(depot['id'], position)
    
    def get_depot_position(self, depot_id) -> LocalPosition:
        for depot in self.depots:
            if depot.id == depot_id:
                # return np arr
                return self.grid_to_local(depot.grid_position)
           
        raise ValueError(f"Depot with id {depot_id} not found.")
            
    def can_reach(self, start, end, drone_radius, connectivity=8):
        # make sure start and end are in grid coordinates
        if isinstance(start, LocalPosition) and isinstance(end, LocalPosition):
            start = self.local_to_grid(start)
            end = self.local_to_grid(end)

        if not (isinstance(start, GridPosition) and isinstance(end, GridPosition)):
            raise TypeError("start and end must be LocalPosition or GridPosition")

        # check if start and end are in bounds
        if not self.is_traversable(start, drone_radius) or not self.is_traversable(end, drone_radius):
            return False

        # simple bfs for reachability
        queue = deque([start])
        visited = {start}

        while queue:
            current = queue.popleft()
            if current == end:
                return True

            for neighbor in self.get_footprint_neighbors(current, drone_radius, connectivity):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                
        return False  
        
    def local_to_grid(self, local_pos: LocalPosition) -> GridPosition:
        # check if inside of map bounds
        if not (0 <= local_pos.x < self.grid_width * self.resolution and 0 <= local_pos.y < self.grid_height * self.resolution):
            raise ValueError(f"Local position {local_pos} is out of bounds in map of size ({self.grid_width * self.resolution}, {self.grid_height * self.resolution})")
        
        return GridPosition(local_pos.x // self.resolution, local_pos.y // self.resolution)
    
    def grid_to_local(self, grid_pos: GridPosition) -> LocalPosition:
        if not (0 <= grid_pos.gx < self.grid_width and 0 <= grid_pos.gy < self.grid_height):
            raise ValueError(f"Grid position {grid_pos} is out of bounds in map of size ({self.grid_width}, {self.grid_height})")
        
        # centered grid point in world coordinates
        return LocalPosition((grid_pos.gx + 0.5) * self.resolution, (grid_pos.gy + 0.5) * self.resolution)

    def is_traversable(self, grid_position: GridPosition, drone_radius):
        if self.in_bounds(grid_position):
            x, y = self.grid_to_world_point(grid_position.gx, grid_position.gy, center=True)
            return self.evaluate_footprint(x, y, drone_radius) < AirspaceType.RESTRICTED.value

    def in_bounds(self, position):
        if isinstance(position, LocalPosition):
            return 0 <= position.x < self.grid_width * self.resolution and 0 <= position.y < self.grid_height * self.resolution
        elif isinstance(position, GridPosition):
            return 0 <= position.gx < self.grid_width and 0 <= position.gy < self.grid_height
        else:
            raise TypeError("position must be LocalPosition or GridPosition")
        
    def get_footprint_neighbors(self, grid_position: GridPosition, drone_radius, connectivity=8):
        if not self.in_bounds(grid_position):
            raise ValueError(
                f"Grid position {grid_position} is out of bounds in map of size ({self.grid_width}, {self.grid_height})"
            )

        gx = grid_position.gx
        gy = grid_position.gy

        cardinal = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in cardinal:
            nxt_gx, nxt_gy = gx + dx, gy + dy
            nxt_pos = GridPosition(nxt_gx, nxt_gy)
            if self.in_bounds(nxt_pos) and self.is_traversable(nxt_pos, drone_radius):
                yield nxt_pos

        # checks corners 
        # ensures fit by validating sides of the corners are also traversable
        if connectivity == 8:
            diagonal = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dx, dy in diagonal:
                nxt_gx, nxt_gy = gx + dx, gy + dy
                nxt_pos = GridPosition(nxt_gx, nxt_gy)

                if not self.in_bounds(nxt_pos):
                    continue

                if not self.is_traversable(nxt_pos, drone_radius):
                    continue

                side_a = GridPosition(gx + dx, gy)
                side_b = GridPosition(gx, gy + dy)

                if self.is_in_bounds(side_a) and self.is_in_bounds(side_b):
                    if self.is_traversable(side_a, drone_radius) and self.is_traversable(side_b, drone_radius):
                        yield nxt_pos