from src.policies import *
from src.wrappers import *
from src.schemas import PolicyType, WrapperType


def create_planner(config):
    # define policy based on config
    if config.policy == PolicyType.GREEDY:
        base_planner = GreedyPolicy(None, config.max_speed, config.safety_radius)
    elif config.policy == PolicyType.ASTAR:
        base_planner = AStarPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.DSTAR:
        base_planner = DStarPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.OCCUPANCY_ASTAR:
        base_planner = OccupancyAStarPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.DFS:
        base_planner = DFSPathPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.BFS:
        base_planner = BFSPathPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.PRIORITIZED_ASTAR:
        base_planner = PrioritizedAStarPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
            time_step=config.reservation_time_step,
            max_time_steps=config.reservation_horizon,
            priority_mode=config.priority_mode,
            priority_buckets=config.priority_buckets,
        )
    elif config.policy == PolicyType.COOPERATIVE_ASTAR:
        base_planner = CooperativeAStarPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
            time_step=config.reservation_time_step,
            max_time_steps=config.reservation_horizon,
            priority_mode=config.priority_mode,
            priority_buckets=config.priority_buckets,
        )
    elif config.policy == PolicyType.WHCA:
        base_planner = WHCAPolicy(
            None,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
            time_step=config.reservation_time_step,
            window_size=config.whca_window_size,
            max_windows=config.whca_max_windows,
            priority_mode=config.priority_mode,
            priority_buckets=config.priority_buckets,
        )
    else:
        raise ValueError(f"unknown policy type: {config.policy}")

    # apply wrapper based on config
    if config.wrapper == WrapperType.NONE:
        return base_planner
    elif config.wrapper == WrapperType.VO:
        return VOWrapper(base_planner, config.safety_radius, active=True)
    else:
        raise ValueError(f"unknown wrapper type: {config.wrapper}")
