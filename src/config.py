from dataclasses import dataclass
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
   
    # job gen rate
    lambda_rate: float = 1/12   # poisson rate (approx 1 job per 12s)
    
    # fleet and safty vars
    fleet_size: int = 4         # number of UAV
    max_speed: float = 5.0      # max UAV speed [m/s]
    safety_radius: float = 2.5  # collision avoidance radius [m]

    # policy and wrapper
    policy: PolicyType = PolicyType.GREEDY
    wrapper: WrapperType = WrapperType.NONE
    connectivity: int = 4

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