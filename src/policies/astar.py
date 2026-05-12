import heapq
import math

import numpy as np

from src.physics import GridPosition, LocalPosition, Velocity
from src.policies.base import MAPFPolicy

class AStarPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5, connectivity=4):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)
        
        # check connectivity is valid
        if connectivity not in (4, 8):
            raise ValueError(f"connectivity must be 4 or 8, got {connectivity}")
        
        self.connectivity = connectivity

    # get heuristic cost
    def _heuristic(self, node, goal):
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        if self.connectivity == 4:
            return float(dx + dy)
        return float(math.hypot(dx, dy))

    # cost to move from cur to next
    def _move_cost(self, current, nxt):
        dx = abs(current[0] - nxt[0])
        dy = abs(current[1] - nxt[1])
        step = math.sqrt(2.0) if dx == 1 and dy == 1 else 1.0
        weight = float(self.grid_map.weighted_grid[nxt[0], nxt[1]])
        return step * weight

    # get path
    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # plan path
    def plan_path(self, start, goal, spatial_manager=None, uav_id=None):
        # doesnt use id
        del uav_id

        # get start and end as position
        start_array = self._position_array(start)
        goal_array = self._position_array(goal)

        start_node = self.grid_map.world_to_grid_point(start_array[0], start_array[1], clamp=True)
        goal_node = self.grid_map.world_to_grid_point(goal_array[0], goal_array[1], clamp=True)

        # if start and end are not traversable return start as only path!
        if not self.grid_map.is_traversable(GridPosition(start_node[0], start_node[1]), self.drone_radius) or not self.grid_map.is_traversable(GridPosition(goal_node[0], goal_node[1]), self.drone_radius):
            return [start]

        open_heap = []
        heapq.heappush(open_heap, (self._heuristic(start_node, goal_node), 0, start_node))

        came_from = {}
        g_score = {start_node: 0.0}
        closed = set()
        counter = 1

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal_node:
                grid_path = self._reconstruct_path(came_from, current)
                world_path = [LocalPosition(float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[0]), float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[1])) for node in grid_path]
                if len(world_path) > 0:
                    world_path[0] = start
                    world_path[-1] = goal
                return world_path

            closed.add(current)

            for nxt in self.grid_map.get_neighbors(current[0], current[1], self.drone_radius, self.connectivity):
                if nxt in closed:
                    continue

                tentative_g = g_score[current] + self._move_cost(current, nxt)
                if tentative_g < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    f_score = tentative_g + self._heuristic(nxt, goal_node)
                    heapq.heappush(open_heap, (f_score, counter, nxt))
                    counter += 1

        return [start]

    # get vel
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