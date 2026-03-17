import simpy
import random
import logging
import numpy as np

from src.entities import UAV
from src.metrics import Metrics

from factory import create_planner

from src.environment.map import GridMap, AirspaceType
from src.environment.spatial import SpatialManager

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
        
        # init map
        self.map = GridMap(
            self.cfg.map_width, 
            self.cfg.map_height, 
            potential_field=self.cfg.potential_field, 
            potential_strength=self.cfg.potential_strength, 
            file=self.cfg.file
        )
        
        # init spatial manager
        self.sm = SpatialManager(self.cfg.safety_radius)
        self.metrics = Metrics() 
        
        # get planner
        self.planner = create_planner(self.cfg, self.map)

        log.info(f"Initialized Planner: {type(self.planner).__name__}")
        if hasattr(self.planner, 'active'):
            if self.planner.active is True:
                log.info("VO safety wrapper is active")
            else:
                log.info("VO safety wrapper is disabled")

        # init fleet
        self.uavs = []
        for i in range(self.cfg.fleet_size):
            # fetch the exact spawn coordinates from the map's depots
            start_x, start_y = self.map.get_depot_spawn(i)
            
            uav = UAV(
                env=self.env,
                id=f"UAV_{i}",
                depot_pos=[start_x, start_y],
                spatial_manager=self.sm,
                metrics=self.metrics,
                policy=self.planner,
                job_queue=self.job_queue,
                cfg=self.cfg,
                dt=self.cfg.dt,
                grid_map=self.map
            )
            self.uavs.append(uav)
            
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
                gx = random.uniform(0, self.cfg.map_width)
                gy = random.uniform(0, self.cfg.map_height)

                # varify safe
                if self.map.evaluate_footprint(gx, gy, 0.5) < AirspaceType.PROHIBITED.value:
                    valid_location = True

            job = {
                'id': job_count,
                'goal': [gx, gy]
            }
            log.info(f"[{self.env.now:4.1f}] Job {job_count} requested at ({gx:.1f}, {gy:.1f}).")
            self.job_queue.put(job)

    # check for conflicts
    def monitor(self):
        while True:
            self.sm.check_conflicts(self.metrics)
            yield self.env.timeout(self.cfg.dt)

    # run simulation
    def run(self):
        log.info(f"Starting Simulation for {self.cfg.sim_time}s...")
        self.env.run(until=self.cfg.sim_time)
        log.info(f"[{self.env.now:4.1f}] END OF SIMULATION")
        
        self.metrics.save_to_csv(self.cfg.csv_file)
        log.info(f"Results saved to {self.cfg.csv_file}")