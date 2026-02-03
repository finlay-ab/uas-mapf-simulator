import simpy
import random
import numpy as np
import logging
from entities import UAV
from metrics import Metrics
from environment import SpatialManager
from policies.greedy import GreedyPolicy

# config
DT = 0.5            # simulation time step [s]
SIM_TIME = 200      # total sim seconds
LAMBDA = 1/12       # poisson rate (aprox 1 job per 12s)
SAFETY_RADIUS = 2.0 
MAX_SPEED = 5.0     
FLEET_SIZE = 4      # number of active UAVs

# set logging
logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    handlers=[
        logging.FileHandler("simulation.log", mode="w"),  # write to file
        logging.StreamHandler()  # print to cli
    ]
)
log = logging.getLogger("UAS_Sim")

def job_generator(env, job_queue, metrics):
    job_count = 0
    while True:
        # poisson inter-arrival time
        yield env.timeout(random.expovariate(LAMBDA))
        job_count += 1
        
        # update metrics count
        metrics.jobs_requested = job_count
        
        job = {
            'id': job_count, 
            'goal': [random.uniform(0, 100), random.uniform(0, 100)]
        }
        log.info(f"[{env.now:4.1f}] Job {job_count} requested.")
        job_queue.put(job)

def monitor(env, sm, metrics):
    while True:
        # check conflicts and update violation count
        sm.check_conflicts(metrics)
        yield env.timeout(DT)

if __name__ == "__main__":
    # set up sim
    env = simpy.Environment()
    sm = SpatialManager(SAFETY_RADIUS)
    met = Metrics()
    pol = GreedyPolicy(MAX_SPEED)
    job_queue = simpy.Store(env)

    # initialize fleet at Depot [10, 10]
    for i in range(FLEET_SIZE):
        UAV(env, f"UAV_{i}", [10, 10], sm, met, pol, job_queue, DT)

    # start processes
    env.process(job_generator(env, job_queue, met))
    env.process(monitor(env, sm, met))
    
    # run sim
    env.run(until=SIM_TIME)
    log.info(f"[{env.now:4.1f}] END OF SIMULATION")
    
    print(f"\nExperiment Results:")
    stats = met.get_summary_statistics()
    for key, value in stats.items():
        print(f"{key}: {value}")

    # save metrics to csv
    met.save_to_csv("run_results.csv")
    log.info("metrics saved to run_results.csv")
    log.info("log saved to simulation.log")