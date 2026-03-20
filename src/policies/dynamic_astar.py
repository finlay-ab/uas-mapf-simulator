import math

from .astar import AStarPolicy

class DynamicAStarPolicy(AStarPolicy):
    def plan_path(self, start, goal, spatial_manager=None):
        # gets uav positions and marks them as blocked
        self.dynamic_blocked = self._build_dynamic_blocked_set(spatial_manager)
  
        # ensures the set of blocked cells doesnt include start and end goals
        start_node = self.grid_map.world_to_grid_point(float(start[0]), float(start[1]), clamp=True)
        goal_node = self.grid_map.world_to_grid_point(float(goal[0]), float(goal[1]), clamp=True)
        self.dynamic_blocked.discard(start_node)
        self.dynamic_blocked.discard(goal_node)
 
        # run A* ensuring grid includes dynamic blocked set
        return super().plan_path(start, goal, spatial_manager)
    
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

        # for all uav poistions mark 
        for _uav_id, position in spatial_manager.positions.items():
            gx, gy = self.grid_map.world_to_grid_point(position[0], position[1], clamp=True)
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
                    