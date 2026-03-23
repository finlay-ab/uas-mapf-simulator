from src.policies import *
from src.wrappers import *
from src.schemas import PolicyType, WrapperType


def create_planner(config, grid_map):
    # define policy based on config
    if config.policy == PolicyType.GREEDY:
        base_planner = GreedyPolicy(grid_map, config.max_speed, config.safety_radius)
    elif config.policy == PolicyType.ASTAR:
        base_planner = AStarPolicy(
            grid_map,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.DSTAR:
        base_planner = DStarPolicy(
            grid_map,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
        )
    elif config.policy == PolicyType.OCCUPANCY_ASTAR:
        base_planner = OccupancyAStarPolicy(
            grid_map,
            config.max_speed,
            config.safety_radius,
            connectivity=config.connectivity,
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