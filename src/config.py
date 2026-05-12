import json
from dataclasses import dataclass, fields
from enum import Enum

from src.schemas import PolicyType, WrapperType, PathRecoveryStrategy, PathRecoveryAction

@dataclass
class SimConfig:
    # sim times
    dt: float = 0.5             # time step [s]
    sim_time: float = 200.0     # total runtime [s]
    
    # env
    map_width: int = 400
    map_height: int = 400
    potential_field: bool = True
    potential_strength: float = 10.0
    file: str = "src/environment/obstacles.txt"

    world_config: str = "config/worlds/default/world_manifest.json"

    waypoint_arrival_threshold: float = 1.5
   
    # job gen rate
    lambda_rate: float = 1/12   # poisson rate (approx 1 job per 12s)
    
    # fleet and safty vars
    fleet_size: int = 4         # number of UAV
    max_speed: float = 5.0      # max UAV speed [m/s]
    safety_radius: float = 2.5  # collision avoidance radius [m]

    # when false uavs will hover wait when a potential collision could happen
    allow_predicted_collisions: bool = True

    # policy and wrapper
    policy: PolicyType = PolicyType.GREEDY
    wrapper: WrapperType = WrapperType.NONE
    connectivity: int = 8
    reservation_time_step: float = None
    reservation_horizon: int = 2000
    whca_window_size: int = 24
    whca_max_windows: int = 12
    priority_mode: str = "none"
    priority_buckets: int = 4
    local_avoidance_on_predicted_collision: bool = False
    local_avoidance_mode: str = "none"

    # path recovery 
    path_recovery_strategy: PathRecoveryStrategy = PathRecoveryStrategy.RETURN_TO_NEXT_WP
    path_recovery_action: PathRecoveryAction = PathRecoveryAction.RETURN_TO_PATH

    off_path_threshold: float = 3.0     # distance (m) from path to trigger recovery
    off_path_check_interval: int = 2    # how many time steps before checking
    max_recovery_attempts: int = 5      # how many attempts before giving up and replanning

    # stuck UAV detection and recovery
    hover_timeout: float = 10.0                              
    job_timeout: float = 300.0                               
    max_waypoint_skips: int = 3                              

    # takeoff and landing
    depot_operation_radius: float = 5.0
    depot_check_interval: float = 0.5

    # out
    log_file: str = "simulation.log"
    csv_file: str = "run_results.csv"

    # random 
    seed: int | None = None

    # load config from dict/ file
    @classmethod
    def from_dict(class_, data):
        # gets str to enum
        enum_fields = {
            "policy": PolicyType,
            "wrapper": WrapperType,
            "path_recovery_strategy": PathRecoveryStrategy,
            "path_recovery_action": PathRecoveryAction,
        }

        valid_keys = {f.name for f in fields(class_)}
        converted = {}

        for key, value in data.items():
            if key not in valid_keys:
                raise ValueError(f"unknown sim config: {key}")

            # turn enum to key 
            if key in enum_fields and isinstance(value, str):
                value = enum_fields[key][value.strip().upper()]

            converted[key] = value

        return class_(**converted)

    @classmethod
    def from_file(class_, config_file, overrides=None):
        with open(config_file, 'r') as f:
            data = json.load(f)

        if overrides:
            data.update(overrides)

        return class_.from_dict(data)