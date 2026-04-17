import simpy
import random
import logging
import numpy as np

from src.entities import UAV
from src.metrics import Metrics

from src.factory import create_planner

from src.environment.map import GridMap, AirspaceType
from src.environment.spatial import SpatialManager
from src.environment.world_manger import WorldManager

# logging
log = logging.getLogger("UAS_Sim")

class Simulation:
    def __init__(self, config):
        self.cfg = config

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
        self.job_queue = simpy.Store(self.env)
        

        # init metrics 
        self.metrics = Metrics()

        # init planner
        self.planner = create_planner(self.cfg)

        # init world manager
        self.world_manager = WorldManager(self.cfg, self.env, self.planner, self.job_queue)
        self.airspace = self.world_manager.get_airspace("airspace1")
        if self.airspace is None:
            raise ValueError("airspace1 not found in world manifest")

        self.sm = self.airspace.spatial_manager
        self.planner.grid_map = self.airspace.map
        # self.sm = SpatialManager(self.cfg, self.env, self.airspace, self.metrics)
        # start processes
        self.env.process(self.job_generator())
        self.env.process(self.monitor())

    # create jobs
    def job_generator(self):
        job_count = 0
        while True:
            yield self.env.timeout(random.expovariate(self.cfg.lambda_rate))
            job_count += 1
            
            self.metrics.record_job_request()
            
            # make sure job is in valid loc
            valid_location = False
            while not valid_location:
                map_width = self.airspace.map.grid_width * self.airspace.map.resolution
                map_height = self.airspace.map.grid_height * self.airspace.map.resolution

                gx = random.uniform(0, map_width)
                gy = random.uniform(0, map_height)

                # varify safe
                if self.airspace.map.evaluate_footprint(gx, gy, 0.5) < AirspaceType.PROHIBITED.value:
                    valid_location = True

            job = {
                'id': job_count,
                'goal': [gx, gy]
            }
            log.info(f"[{self.env.now:4.1f}] Job {job_count} requested at ({gx:.1f}, {gy:.1f}).")
            self.job_queue.put(job)

    # check for conflicts
    def monitor(self):
        while False:
            self.sm.check_conflicts(self.metrics)
            yield self.env.timeout(self.cfg.dt)

    # run simulation
    def run(self):
        # log start
        log.info(f"Starting Simulation for {self.cfg.sim_time}s...")
        
        # run simulation
        self.env.run(until=self.cfg.sim_time)

        # log end
        log.info(f"[{self.env.now:4.1f}] END OF SIMULATION")
        
        # save metrics
        self.metrics.save_to_csv(self.cfg.csv_file)
        log.info(f"Results saved to {self.cfg.csv_file}")