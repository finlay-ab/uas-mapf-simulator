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


    #    log.info(f"Initialized Planner: {type(self.planner).__name__}")
    #    if hasattr(self.planner, 'active'):
    #        if self.planner.active is True:
    #            log.info("VO safety wrapper is active")
    #        else:
    #            log.info("VO safety wrapper is disabled")
