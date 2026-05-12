from src.policies.prioritized_astar import PrioritizedAStarPolicy


# cooperative a* is prioritized a* with a longer goal and a deterministic priority
class CooperativeAStarPolicy(PrioritizedAStarPolicy):
    def __init__(
        self,
        grid_map,
        max_speed=5.0,
        safety_radius=1.0,
        drone_radius=0.5,
        connectivity=4,
        time_step=None,
        max_time_steps=200,
        priority_mode="lexicographic",
        priority_buckets=4,
    ):
        
        super().__init__(
            grid_map,
            max_speed=max_speed,
            safety_radius=safety_radius,
            drone_radius=drone_radius,
            connectivity=connectivity,
            time_step=time_step,
            max_time_steps=max_time_steps,
            priority_mode=priority_mode,
            priority_buckets=priority_buckets,
            goal_hold_steps=4,
        )

    