import math
import logging

from .astar import AStarPolicy
from src.helpers import to_array

log = logging.getLogger("UAS_Sim")

MAX_EXPANSIONS = 250_000


class OccupancyAStarPolicy(AStarPolicy):
    def __init__(self, grid_map, max_speed=5.0, safety_radius=1.0, drone_radius=0.5, connectivity=4):
        # override A* vars
        super().__init__(grid_map, max_speed, safety_radius, drone_radius, connectivity)
        self.max_expansions = MAX_EXPANSIONS

    def plan_path(self, start, goal, spatial_manager=None, uav_id=None):
        del uav_id
        # gets uav positions and marks them as blocked
        self.dynamic_blocked = self._build_dynamic_blocked_set(spatial_manager)

        # ensures not blocking start and end cells
        start_array = self._position_array(start)
        goal_array = self._position_array(goal)
        start_node = self.grid_map.world_to_grid_point(float(start_array[0]), float(start_array[1]), clamp=True)
        goal_node = self.grid_map.world_to_grid_point(float(goal_array[0]), float(goal_array[1]), clamp=True)
        self.dynamic_blocked.discard(start_node)
        self.dynamic_blocked.discard(goal_node)

        # run A* 
        path = super().plan_path(start, goal, spatial_manager)
        if path is not None and len(path) >= 2:
            return path

        # fail use A* result
        log.debug("occupancy astar fallback to static path for start=%s goal=%s", start, goal)
        saved_blocked = self.dynamic_blocked
        self.dynamic_blocked = set()
        
        try:
            return AStarPolicy.plan_path(self, start, goal, spatial_manager, None)
        finally:
            self.dynamic_blocked = saved_blocked

    # set move cost to inf if in blocked set
    def _move_cost(self, current, nxt):
        if nxt in self.dynamic_blocked:
            return float("inf")
        return super()._move_cost(current, nxt)

    # build blocked set from spatial manager positions
    def _build_dynamic_blocked_set(self, spatial_manager):
        blocked = set()

        # if no spatial manager or no positions then
        if spatial_manager is None or not hasattr(spatial_manager, "positions"):
            return blocked

        # imported locally to avoid crashing
        from src.entities import UAVState
        ground_states = (UAVState.IDLE_DEPOT, UAVState.TAKEOFF, UAVState.LANDING)

        # for each pos add to blocked set
        for uav_id, position in spatial_manager.positions.items():
            uav_state = spatial_manager.states.get(uav_id)
            if uav_state in ground_states:
                continue
            pos_array = to_array(position)
            gx, gy = self.grid_map.world_to_grid_point(pos_array[0], pos_array[1], clamp=True)
            self._mark_radius_blocked(blocked, gx, gy, self.safety_radius)
        return blocked

    # block out radius of drone in grid
    def _mark_radius_blocked(self, blocked_set, center_x, center_y, radius):
        if radius <= 0:
            blocked_set.add((center_x, center_y))
            return

        grid_radius = int(math.ceil(radius / self.grid_map.resolution))
        for gx in range(max(0, center_x - grid_radius), min(self.grid_map.grid_width, center_x + grid_radius + 1)):
            for gy in range(max(0, center_y - grid_radius), min(self.grid_map.grid_height, center_y + grid_radius + 1)):
                if math.hypot(gx - center_x, gy - center_y) <= grid_radius:
                    blocked_set.add((gx, gy))
