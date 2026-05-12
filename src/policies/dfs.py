from collections import deque

import numpy as np

from src.physics import GridPosition, LocalPosition, Velocity
from src.policies.base import MAPFPolicy

# hard cap on node expansions to stop pathological searches running forever
MAX_EXPANSIONS = 200_000


class BFSPathPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5, connectivity=4):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)

        if connectivity not in (4, 8):
            raise ValueError(f"connectivity must be 4 or 8, got {connectivity}")
        self.connectivity = connectivity

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plan_path(self, start, goal, spatial_manager=None, uav_id=None):
        del spatial_manager
        del uav_id

        if self.grid_map is None:
            return None

        start_array = self._position_array(start)
        goal_array = self._position_array(goal)

        start_node = self.grid_map.world_to_grid_point(start_array[0], start_array[1], clamp=True)
        goal_node = self.grid_map.world_to_grid_point(goal_array[0], goal_array[1], clamp=True)

        # start or goal blocked: no path possible
        if not self.grid_map.is_traversable(GridPosition(start_node[0], start_node[1]), self.drone_radius):
            return None
        if not self.grid_map.is_traversable(GridPosition(goal_node[0], goal_node[1]), self.drone_radius):
            return None

        queue = deque([start_node])
        came_from = {}
        visited = {start_node}
        expansions = 0

        while queue:
            current = queue.popleft()
            if current == goal_node:
                grid_path = self._reconstruct_path(came_from, current)
                world_path = [LocalPosition(float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[0]), float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[1])) for node in grid_path]
                if world_path:
                    world_path[0] = start
                    world_path[-1] = goal
                return world_path

            # stop if we have expanded too many nodes
            expansions += 1
            if expansions >= MAX_EXPANSIONS:
                return None

            for nxt in self.grid_map.get_neighbors(current[0], current[1], self.drone_radius, self.connectivity):
                if nxt in visited:
                    continue
                visited.add(nxt)
                came_from[nxt] = current
                queue.append(nxt)

        # queue exhausted without finding the goal
        return None

    def get_velocity(self, name, pos, target, spatial_manager):
        del name
        del spatial_manager

        pos_array = self._position_array(pos)
        target_array = self._position_array(target)
        direction = target_array - pos_array
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return Velocity(0.0, 0.0)

        unit_direction = direction / distance
        speed = min(self.max_speed, distance)
        velocity = unit_direction * speed
        return Velocity(float(velocity[0]), float(velocity[1]))