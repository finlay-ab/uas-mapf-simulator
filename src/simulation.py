import simpy
import random
import logging
import numpy as np

from src.metrics import Metrics
from src.factory import create_planner

from src.environment.world_manger import WorldManager

log = logging.getLogger("UAS_Sim")

class Simulation:
    def __init__(self, config):
        self.cfg = config

        # use seed
        if self.cfg.seed is not None:
            random.seed(int(self.cfg.seed))
            np.random.seed(int(self.cfg.seed))

        # setup logging
        logging.basicConfig(
            level=logging.INFO,
            format="%(message)s",
            handlers=[
                logging.FileHandler(self.cfg.log_file, mode='w'),
                logging.StreamHandler()                          
            ],
            # over write tests
            force=True 
        )
        
        # create env
        self.env = simpy.Environment()

        # init metrics 
        self.metrics = Metrics()

        # init planner
        self.planner = create_planner(self.cfg)

        # init world manager
        self.world_manager = WorldManager(self.cfg, self.env, self.planner, self.metrics)


    # run simulation
    def run(self):
        log.info("="*80)
        log.info(f"Starting Simulation for {self.cfg.sim_time}s")
        log.info("="*80)
        
        self.env.run(until=self.cfg.sim_time)
        
        log.info(f"[{self.env.now:4.1f}] END OF SIMULATION")
        self.metrics.save_to_csv(self.cfg.csv_file)
        log.info(f"Results saved to {self.cfg.csv_file}")


def simulation_from_config_file(config_file, overrides=None):
    from src.config import SimConfig

    cfg = SimConfig.from_file(config_file, overrides=overrides)
    return Simulation(cfg)