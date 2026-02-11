import simpy
import random
import logging
import numpy as np

from src.entities import UAV
from src.metrics import Metrics
from src.environment import SpatialManager
from src.policies.greedy import GreedyPolicy
from src.policies.vo_policy import VOPolicy

# logging
log = logging.getLogger("UAS_Sim")

class Simulation:
    def __init__(self, config):
        self.cfg = config
        
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
        
        self.cfg = config
        self.env = simpy.Environment()
        self.job_queue = simpy.Store(self.env)
        
        # init 
        self.sm = SpatialManager(self.cfg.safety_radius)
        self.metrics = Metrics() 
        
        # get policy
        if self.cfg.policy == "greedy":
            self.policy = GreedyPolicy(self.cfg.max_speed)
            log.info("Active Policy: Greedy (baseline - no avoidance)")
        elif self.cfg.policy == "vo":
            self.policy = VOPolicy(self.cfg.max_speed, self.cfg.safety_radius)
            log.info("Active Policy: VO Policy (tactical avoidance)")
        else:
            raise ValueError(f"unknown policy type: {self.cfg.policy}")

        # init fleet
        self.uavs = []
        for i in range(self.cfg.fleet_size):
            uav = UAV(
                env=self.env,
                id=f"UAV_{i}",
                depot_pos=[10, 10],
                spartial_manager=self.sm,
                metrics=self.metrics,
                policy=self.policy,
                job_queue=self.job_queue,
                dt=self.cfg.dt
            )
            self.uavs.append(uav)
            
        # start prosesses
        self.env.process(self.job_generator())
        self.env.process(self.monitor())

    # create jobs
    def job_generator(self):
        job_count = 0
        while True:
            yield self.env.timeout(random.expovariate(self.cfg.lambda_rate))
            job_count += 1
            
            self.metrics.record_job_request()
            
            job = {
                'id': job_count,
                'goal': [random.uniform(0, self.cfg.size), random.uniform(0, self.cfg.size)]
            }
            log.info(f"[{self.env.now:4.1f}] Job {job_count} requested.")
            self.job_queue.put(job)

    # check for conflics
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