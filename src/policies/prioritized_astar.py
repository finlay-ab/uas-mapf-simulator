import heapq
import logging
import math

import numpy as np

from src.physics import GridPosition, LocalPosition, Velocity
from src.policies.astar import AStarPolicy

log = logging.getLogger("UAS_Sim")

MAX_EXPANSIONS = 250000

class PrioritizedAStarPolicy(AStarPolicy):
    def __init__(
        self,
        grid_map,
        max_speed=5.0,
        safety_radius=1.0,
        drone_radius=0.5,
        connectivity=4,
        time_step=None,
        max_time_steps=200,
        priority_mode="hash",
        priority_buckets=4,
        goal_hold_steps=2,
    ):
        super().__init__(grid_map, max_speed, safety_radius, drone_radius, connectivity)
        self.max_expansions = MAX_EXPANSIONS
        self.time_step = time_step
        self.max_time_steps = max_time_steps
        self.priority_mode = priority_mode
        self.priority_buckets = max(1, int(priority_buckets))
        self.goal_hold_steps = max(0, int(goal_hold_steps))

    # get sim second to time-step
    def _time_index(self, time_s, step_s):
        if step_s <= 0:
            return 0
        return int(math.floor(time_s / step_s))

    # adds a time to lower priority uavs
    def _priority_offset(self, uav_id):
        if uav_id is None or self.priority_mode == "none":
            return 0

        # set priority 
        token = str(uav_id)
        if self.priority_mode == "lexicographic":
            return sum(ord(ch) for ch in token) % self.priority_buckets
        
        if self.priority_mode == "reverse_lexicographic":
            return (self.priority_buckets - 1) - (sum(ord(ch) for ch in token) % self.priority_buckets)
        
        if self.priority_mode == "hash":
            return hash(token) % self.priority_buckets
            
        return 0

    # get path from nodes
    def _state_path_nodes(self, state, came_from):
        nodes = [state[0]]
        curr = state
        while curr in came_from:
            curr = came_from[curr]
            nodes.append(curr[0])
        nodes.reverse()
        return nodes

    # reserve path
    def _reserve_nodes(self, path_nodes, spatial_manager, uav_id, start_t):
        t = start_t
        prev = None
        for node in path_nodes:
            spatial_manager.reserve_cell(uav_id, node, t)
            
            if prev is not None:
                if hasattr(spatial_manager, "reserve_edge"):
                    spatial_manager.reserve_edge(uav_id, prev, node, t)
                    
            prev = node
            t += 1

    # plan path
    def _plan_nodes(self, start_node, goal_node, spatial_manager, uav_id, start_t, horizon_steps, return_partial=False):
        open_list = []
        count = 0
        start_state = (start_node, start_t)
        
        g_score = {start_state: 0.0}
        came_from = {}
        visited = set()

        # track closest path for partials
        best_state = start_state
        best_h = self._heuristic(start_node, goal_node)

        heapq.heappush(open_list, (best_h, count, start_node, start_t))
        count += 1

        expansions = 0

        while open_list:
            _, _, curr, t_idx = heapq.heappop(open_list)
            state = (curr, t_idx)
            
            if state in visited:
                continue
            visited.add(state)

            # update closest path for partials
            h = self._heuristic(curr, goal_node)
            if h < best_h:
                best_h = h
                best_state = state

            if curr == goal_node:
                return self._state_path_nodes(state, came_from), True

            # check expansion budget
            expansions += 1
            if expansions >= self.max_expansions:
                log.warning(f"PrioritizedA* fail: hit max_expansions={self.max_expansions} for uav {uav_id}")
                break

            # check time horizon
            if t_idx - start_t >= horizon_steps:
                continue

            neighbors = list(self.grid_map.get_neighbors(curr[0], curr[1], self.drone_radius, self.connectivity))
            neighbors.append(curr)

            for nxt in neighbors:
                next_t = t_idx + 1
                
                # check vertex reservation
                if spatial_manager.is_reserved(nxt, next_t, uav_id=uav_id):
                    continue
                    
                # check swap conflict
                if hasattr(spatial_manager, "is_swap_conflict"):
                    if spatial_manager.is_swap_conflict(curr, nxt, next_t, uav_id=uav_id):
                        continue

                new_g = g_score[state] + 1.0
                next_state = (nxt, next_t)
                
                if next_state not in g_score or new_g < g_score[next_state]:
                    came_from[next_state] = state
                    g_score[next_state] = new_g
                    f_score = new_g + self._heuristic(nxt, goal_node)
                    heapq.heappush(open_list, (f_score, count, nxt, next_t))
                    count += 1

        # if goal not met
        if return_partial and best_state != start_state:
            return self._state_path_nodes(best_state, came_from), False

        return None, False

    def plan_path(self, start, goal, spatial_manager=None, uav_id=None):
        if not self.grid_map:
            return None

        # fall back to A* with no spartial
        if not spatial_manager or uav_id is None or not hasattr(spatial_manager, "reserve_cell"):
            return super().plan_path(start, goal, spatial_manager)

        start_arr = self._position_array(start)
        goal_arr = self._position_array(goal)

        start_node = self.grid_map.world_to_grid_point(start_arr[0], start_arr[1], clamp=True)
        goal_node = self.grid_map.world_to_grid_point(goal_arr[0], goal_arr[1], clamp=True)

        # check if traversable
        if not self.grid_map.is_traversable(GridPosition(start_node[0], start_node[1]), self.drone_radius):
            log.warning(f"PrioritizedA* fail: start {start_node} not traversable")
            return None
            
        if not self.grid_map.is_traversable(GridPosition(goal_node[0], goal_node[1]), self.drone_radius):
            log.warning(f"PrioritizedA* fail: goal {goal_node} not traversable")
            return None

        # get time lenght
        step_s = self.time_step
        if not step_s:
            step_s = max(self.grid_map.resolution / max(self.max_speed, 1e-6), 1e-3)

        # current sim time becomes start
        start_time = 0.0
        if spatial_manager.env:
            start_time = float(spatial_manager.env.now)
            
        start_t = self._time_index(start_time, step_s) + self._priority_offset(uav_id)

        # check for expired reservations
        if hasattr(spatial_manager, "expire_stale_reservations"):
            spatial_manager.expire_stale_reservations(start_t)

        # clear any reservations from this uavs previous plans
        spatial_manager.clear_reservations(uav_id)

        path_nodes, reached_goal = self._plan_nodes(
            start_node, goal_node, spatial_manager, uav_id, start_t, self.max_time_steps
        )

        if not path_nodes or len(path_nodes) < 2:
            log.warning(f"prioritizedA* fail: no path for uav {uav_id} from {start_node} to {goal_node}")
            return None

        # add to spartial mangaer
        self._reserve_nodes(path_nodes, spatial_manager, uav_id, start_t)

        #  reserve goal
        if self.goal_hold_steps > 0:
            hold_node = goal_node if reached_goal else path_nodes[-1]
            hold_start = start_t + len(path_nodes)
            for step in range(self.goal_hold_steps):
                spatial_manager.reserve_cell(uav_id, hold_node, hold_start + step)

        # convert pathto world pos
        world_path = []
        for node in path_nodes:
            wx, wy = self.grid_map.grid_to_world_point(node[0], node[1], center=True)
            world_path.append(LocalPosition(float(wx), float(wy)))
            
        world_path[0] = start
        if reached_goal:
            world_path[-1] = goal
            
        return world_path

    def get_velocity(self, name, pos, target, spatial_manager):
        return super().get_velocity(name, pos, target, spatial_manager)