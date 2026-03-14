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
    def __init__(self, width, height, resolution=1.0, potential_field=False, potential_strength=10.0, file=None):
        self.resolution = resolution
        self.grid_width = int(np.ceil(width / self.resolution))
        self.grid_height = int(np.ceil(height / self.resolution))
        self.potential_field = potential_field
        self.potential_strength = potential_strength

        # initilize grids
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=int)
        
        # initilize weight map
        self.weighted_grid = np.ones((self.grid_width, self.grid_height), dtype=float)

        # store depots
        self.depots = []
        self.depot_counter = 0

        # load obstacles and restricted areas and depots
        if file is not None:
            self.load_from_file(file)

        #if no file was given or file had no depots, make a default one
        if len(self.depots) == 0:
            log.warning("No depots found. Creating default depot at origin.")
            self.add_depot(0, width/2, height/2)

        if self.potential_field:
            self.apply_potential_field(self.potential_strength)

    # convert floating co-ordinates into grid co ordinates
    def get_grid_indices(self, x_min, x_max, y_min, y_max):
        gx_min = max(0, int(x_min / self.resolution))
        gx_max = min(self.grid_width, int(np.ceil(x_max / self.resolution)))
        gy_min = max(0, int(y_min / self.resolution))
        gy_max = min(self.grid_height, int(np.ceil(y_max / self.resolution)))
        
        return gx_min, gx_max, gy_min, gy_max

    # makes rect obstacle in grid
    def add_obstacle(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.get_grid_indices(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.OBSTACLE.value
    
    def add_restricted_area(self, x_min, x_max, y_min, y_max):
        gx_min, gx_max, gy_min, gy_max = self.get_grid_indices(x_min, x_max, y_min, y_max)
        self.grid[gx_min:gx_max, gy_min:gy_max] = AirspaceType.RESTRICTED.value

    # adds a depot location
    def add_depot(self, x_min, x_max, y_min, y_max):
        # calculate center of the depot box
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        
        new_depot = Depot(self.depot_counter, center_x, center_y)
        self.depots.append(new_depot)
        self.depot_counter += 1
    
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

    # safely fetch a depot location for drones to start at
    def get_depot_spawn(self, index=0):
        if len(self.depots) == 0:
            return 0.0, 0.0
        
        # use modulo so if you have 10 drones and 2 depots, they alternate safely
        depot = self.depots[index % len(self.depots)]
        return depot.x, depot.y