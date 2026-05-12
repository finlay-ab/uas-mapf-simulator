import logging

import numpy as np

from src.physics import LocalPosition
from src.policies.prioritized_astar import PrioritizedAStarPolicy

log = logging.getLogger("UAS_Sim")

class WHCAPolicy(PrioritizedAStarPolicy):
    def __init__(
        self,
        grid_map,
        max_speed=5.0,
        safety_radius=1.0,
        drone_radius=0.5,
        connectivity=4,
        time_step=None,
        window_size=24,
        max_windows=12,
        priority_mode="hash",
        priority_buckets=4,
    ):
        super().__init__(
            grid_map,
            max_speed=max_speed,
            safety_radius=safety_radius,
            drone_radius=drone_radius,
            connectivity=connectivity,
            time_step=time_step,
            max_time_steps=max(1, int(window_size)),
            priority_mode=priority_mode,
            priority_buckets=priority_buckets,
            goal_hold_steps=1,
        )
        self.window_size = max(1, int(window_size))
        self.max_windows = max(1, int(max_windows))

    def plan_path(self, start, goal, spatial_manager=None, uav_id=None):
        if not self.grid_map:
            return None

        # fall back to  a* if reservation not available
        if not spatial_manager or uav_id is None or not hasattr(spatial_manager, "reserve_cell"):
            return super().plan_path(start, goal, spatial_manager, uav_id)

        start_array = self._position_array(start)
        goal_array = self._position_array(goal)

        start_node = self.grid_map.world_to_grid_point(start_array[0], start_array[1], clamp=True)
        goal_node = self.grid_map.world_to_grid_point(goal_array[0], goal_array[1], clamp=True)

        # time step length
        step_s = self.time_step
        if step_s is None:
            step_s = max(self.grid_map.resolution / max(self.max_speed, 1e-6), 1e-3)

        # convert current sim time to time step
        start_time = 0.0
        if spatial_manager.env is not None:
            start_time = float(spatial_manager.env.now)
        current_t = self._time_index(start_time, step_s) + self._priority_offset(uav_id)

        # clear old reservations
        spatial_manager.clear_reservations(uav_id)

        current = start_node
        full_nodes = [current]

        # itterativly plan window
        for _ in range(self.max_windows):
            if current == goal_node:
                break

            segment_nodes, reached_goal = self._plan_nodes(
                current,
                goal_node,
                spatial_manager,
                uav_id,
                current_t,
                self.window_size,
                return_partial=True,
            )

            # no path for window
            if segment_nodes is None or len(segment_nodes) <= 1:
                break

            # reserve node
            self._reserve_nodes(segment_nodes, spatial_manager, uav_id, current_t)

            # append the new nodes
            if len(full_nodes) > 0:
                full_nodes.extend(segment_nodes[1:])
            else:
                full_nodes.extend(segment_nodes)

            current = full_nodes[-1]
            current_t += len(segment_nodes) - 1

            if reached_goal:
                break

        if len(full_nodes) <= 1:
            log.warning("WHCA fail: no progress for uav %s from %s to %s", uav_id, start_node, goal_node)
            return None

        world_path = [
            LocalPosition(float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[0]), float(self.grid_map.grid_to_world_point(node[0], node[1], center=True)[1]))
            for node in full_nodes
        ]
        world_path[0] = start
        if full_nodes[-1] == goal_node:
            world_path[-1] = goal
        return world_path