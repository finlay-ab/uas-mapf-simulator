from dataclasses import dataclass

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
   
    # job gen rate
    lambda_rate: float = 1/12   # poisson rate (approx 1 job per 12s)
    
    # fleet and safty vars
    fleet_size: int = 4         # number of UAV
    max_speed: float = 5.0      # max UAV speed [m/s]
    safety_radius: float = 2.5  # collision avoidance radius [m]
    
    # policy vars
    policy: str = "vo"          
    use_vo = True
    
    # out
    log_file: str = "simulation.log"
    csv_file: str = "run_results.csv"