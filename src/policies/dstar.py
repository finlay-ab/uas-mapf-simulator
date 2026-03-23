import heapq
import math
from collections import defaultdict

import numpy as np

from src.physics import Velocity
from src.policies.base import MAPFPolicy

# d* lite policy
class DStarPolicy(MAPFPolicy):
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5, connectivity=4):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius)

        # check connectivity is valid
        if connectivity not in (4, 8):
            raise ValueError(f"connectivity must be 4 or 8, got {connectivity}")

        self.connectivity = connectivity

        # d* lite state
        self.k_m = 0.0
        self.s_start = None
        self.s_goal = None
        self.s_last = None

        # g and rhs maps
        self.g = defaultdict(lambda: float("inf"))
        self.rhs = defaultdict(lambda: float("inf"))

        # priority queue
        self.open_heap = []
        self.open_entries = {}
        self.push_counter = 0

        # occupancy states
        self.dynamic_blocked = set()
        self.last_dynamic_blocked = set()

    # get heuristic cost
    def _heuristic(self, node, goal):
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        if self.connectivity == 4:
            return float(dx + dy)
        return float(math.hypot(dx, dy))

    # cost to move from cur to next
    def _move_cost(self, current, nxt):
        if nxt in self.dynamic_blocked:
            return float("inf")

        dx = abs(current[0] - nxt[0])
        dy = abs(current[1] - nxt[1])
        step = math.sqrt(2.0) if dx == 1 and dy == 1 else 1.0
        weight = float(self.grid_map.weighted_grid[nxt[0], nxt[1]])
        return step * weight

    # build blocked set from other uav positions
    def _build_dynamic_blocked_set(self, spatial_manager):
        blocked = set()
        if spatial_manager is None or not hasattr(spatial_manager, "positions"):
            return blocked

        for _uav_id, position in spatial_manager.positions.items():
            gx, gy = self.grid_map.world_to_grid_point(position[0], position[1], clamp=True)
            self._mark_radius_blocked(blocked, gx, gy, self.safety_radius)
        return blocked

    # block out radius around drone
    def _mark_radius_blocked(self, blocked_set, center_x, center_y, radius):
        if radius <= 0:
            blocked_set.add((center_x, center_y))
            return

        # block all cells in radius of uav
        grid_radius = int(math.ceil(radius / self.grid_map.resolution))
        for gx in range(max(0, center_x - grid_radius), min(self.grid_map.grid_width, center_x + grid_radius + 1)):
            for gy in range(max(0, center_y - grid_radius), min(self.grid_map.grid_height, center_y + grid_radius + 1)):
                if math.hypot(gx - center_x, gy - center_y) <= grid_radius:
                    blocked_set.add((gx, gy))

    # key for open list ordering
    def _key(self, node):
        g_rhs = min(self.g[node], self.rhs[node])
        return (g_rhs + self._heuristic(self.s_start, node) + self.k_m, g_rhs)

    # checks both map traversability and dynamic blocks
    def _is_traversable_node(self, node):
        return self.grid_map.is_traversable(node[0], node[1], self.drone_radius) and node not in self.dynamic_blocked

    # get neighbours respecting connectivity
    def _successors(self, node):
        for nxt in self.grid_map.get_neighbors(node[0], node[1], self.drone_radius, self.connectivity):
            if nxt in self.dynamic_blocked:
                continue
            yield nxt

    def _predecessors(self, node):
        # for this grid predecessor and successor are same
        return self._successors(node)

    # check if node is consistent
    def _is_consistent(self, node):
        return math.isclose(self.g[node], self.rhs[node], rel_tol=1e-9, abs_tol=1e-9)

    # compare tuple keys
    def _key_less(self, a, b):
        if a[0] < b[0] - 1e-9:
            return True
        if abs(a[0] - b[0]) <= 1e-9 and a[1] < b[1] - 1e-9:
            return True
        return False

    # push node into open set
    def _push_open(self, node, key):
        entry = (key[0], key[1], self.push_counter, node)
        self.push_counter += 1
        self.open_entries[node] = entry
        heapq.heappush(self.open_heap, entry)

    # remove node from open set
    def _remove_open(self, node):
        if node in self.open_entries:
            del self.open_entries[node]

    # pop next valid entry
    def _pop_open(self):
        while self.open_heap:
            k1, k2, ctr, node = heapq.heappop(self.open_heap)
            current = self.open_entries.get(node)
            if current is None:
                continue
            if current[0] != k1 or current[1] != k2 or current[2] != ctr:
                continue
            del self.open_entries[node]
            return (k1, k2), node
        return (float("inf"), float("inf")), None

    # get current top key
    def _top_key(self):
        while self.open_heap:
            k1, k2, ctr, node = self.open_heap[0]
            current = self.open_entries.get(node)
            if current is None or current[0] != k1 or current[1] != k2 or current[2] != ctr:
                heapq.heappop(self.open_heap)
                continue
            return (k1, k2)
        return (float("inf"), float("inf"))

    # update rhs and open list state for node
    def _update_vertex(self, node):
        if node != self.s_goal:
            best_rhs = float("inf")
            for nxt in self._successors(node):
                best_rhs = min(best_rhs, self._move_cost(node, nxt) + self.g[nxt])
            self.rhs[node] = best_rhs

        self._remove_open(node)
        if not self._is_consistent(node):
            self._push_open(node, self._key(node))

    # init d* lite state for a new query
    def _initialize(self, start_node, goal_node):
        self.k_m = 0.0
        self.s_start = start_node
        self.s_goal = goal_node
        self.s_last = start_node

        self.g = defaultdict(lambda: float("inf"))
        self.rhs = defaultdict(lambda: float("inf"))

        self.open_heap = []
        self.open_entries = {}
        self.push_counter = 0

        self.rhs[self.s_goal] = 0.0
        self._push_open(self.s_goal, self._key(self.s_goal))

    # main d* loop
    def _compute_shortest_path(self):
        while self._key_less(self._top_key(), self._key(self.s_start)) or not self._is_consistent(self.s_start):
            k_old, u = self._pop_open()
            if u is None:
                break

            k_new = self._key(u)
            if self._key_less(k_old, k_new):
                self._push_open(u, k_new)
                continue

            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for pred in self._predecessors(u):
                    self._update_vertex(pred)
            else:
                self.g[u] = float("inf")
                self._update_vertex(u)
                for pred in self._predecessors(u):
                    self._update_vertex(pred)

    # apply changes from moved start or blocked set updates
    def _apply_environment_changes(self, new_start, new_goal):
        if self.s_start is None or self.s_goal is None or new_goal != self.s_goal:
            self._initialize(new_start, new_goal)
            return

        if new_start != self.s_start:
            self.k_m += self._heuristic(self.s_last, new_start)
            self.s_start = new_start
            self.s_last = new_start

        changed_nodes = self.dynamic_blocked.symmetric_difference(self.last_dynamic_blocked)
        if not changed_nodes:
            return

        affected = set(changed_nodes)
        for node in changed_nodes:
            for pred in self._predecessors(node):
                affected.add(pred)

        for node in affected:
            self._update_vertex(node)

    # get path from current g-values
    def _extract_path(self, start_node, goal_node, start_array):
        path_nodes = [start_node]
        current = start_node
        max_steps = self.grid_map.grid_width * self.grid_map.grid_height
        visited = {start_node}

        for _ in range(max_steps):
            if current == goal_node:
                break

            best_next = None
            best_cost = float("inf")
            for nxt in self._successors(current):
                candidate = self._move_cost(current, nxt) + self.g[nxt]
                if candidate < best_cost:
                    best_cost = candidate
                    best_next = nxt

            if best_next is None or math.isinf(best_cost):
                return [start_array]

            path_nodes.append(best_next)
            current = best_next
            if current in visited and current != goal_node:
                return [start_array]
            visited.add(current)

        if path_nodes[-1] != goal_node:
            return [start_array]

        return [
            np.array(self.grid_map.grid_to_world_point(node[0], node[1], center=True), dtype=float)
            for node in path_nodes
        ]

    # plan path and reuse previous search where possible
    def plan_path(self, start, goal, spatial_manager=None):
        start_array = np.array(start, dtype=float)
        goal_array = np.array(goal, dtype=float)

        start_node = self.grid_map.world_to_grid_point(start_array[0], start_array[1], clamp=True)
        goal_node = self.grid_map.world_to_grid_point(goal_array[0], goal_array[1], clamp=True)

        # get latest blocked cells from current uav positions
        self.last_dynamic_blocked = self.dynamic_blocked
        self.dynamic_blocked = self._build_dynamic_blocked_set(spatial_manager)
        self.dynamic_blocked.discard(start_node)
        self.dynamic_blocked.discard(goal_node)

        # if start or goal invalid then no path
        if not self._is_traversable_node(start_node) or not self._is_traversable_node(goal_node):
            return [start_array]

        # update and repair shortest path tree
        self._apply_environment_changes(start_node, goal_node)
        self._compute_shortest_path()

        # no valid route from start
        if math.isinf(self.g[start_node]) and math.isinf(self.rhs[start_node]):
            return [start_array]

        world_path = self._extract_path(start_node, goal_node, start_array)
        if len(world_path) == 1:
            return world_path

        if len(world_path) > 0:
            world_path[0] = start_array
            world_path[-1] = goal_array
        return world_path

    # get vel
    def get_velocity(self, name, pos, target, spatial_manager):
        del name
        del spatial_manager

        pos_array = np.array(pos, dtype=float)
        target_array = np.array(target, dtype=float)
        direction = target_array - pos_array
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return Velocity(0.0, 0.0)

        unit_direction = direction / distance
        speed = min(self.max_speed, distance)
        velocity = unit_direction * speed
        return Velocity(float(velocity[0]), float(velocity[1]))