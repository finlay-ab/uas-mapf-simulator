import simpy
import random
import numpy as np
import logging
from environment import SpatialManager, Metrics
from policies.greedy import GreedyPolicy

# config
DT = 0.5            # simulation time step [s]
SIM_TIME = 200      # total sim seconds
LAMBDA = 1/12       # poisson rate (aprox 1 job per 12s)
SAFETY_RADIUS = 2.0 
MAX_SPEED = 5.0     

# set logging
logging.basicConfig(level=logging.INFO, format="%(message)s")
log = logging.getLogger("UAS_Sim")

# drone process
def drone_process(env, name, start, goal, sm, metrics, policy):
    pos = np.array(start, dtype=float)
    target = np.array(goal, dtype=float)
    
    # while not within 0.5 units of target
    while np.linalg.norm(target - pos) > 0.5:
        metrics.record_path(name, pos)
        
        # get velocity 
        vel = policy.get_velocity(name, pos, target, sm)
        
        pos += vel * DT
        sm.update(name, pos)
        yield env.timeout(DT)
    
    metrics.completed_deliveries += 1
    log.info(f"[{env.now:4.1f}] {name} arrived.")

def job_generator(env, sm, metrics, policy):
    job_count = 0
    while True:
        # poisson inter-arrival time
        yield env.timeout(random.expovariate(LAMBDA))
        job_count += 1
        name = f"job: {job_count}"
        log.info(f"[{env.now:4.1f}] {name} created and starting flight.")
        start = [random.uniform(0, 100), random.uniform(0, 100)]
        goal = [random.uniform(0, 100), random.uniform(0, 100)]
        env.process(drone_process(env, name, start, goal, sm, metrics, policy))

def monitor(env, sm, metrics):
    while True:
        if sm.check_conflicts():
            metrics.total_violations += 1
        yield env.timeout(DT)

if __name__ == "__main__":
    env = simpy.Environment()
    sm = SpatialManager(SAFETY_RADIUS)
    met = Metrics()
    pol = GreedyPolicy(MAX_SPEED)

    env.process(job_generator(env, sm, met, pol))
    env.process(monitor(env, sm, met))
    
    env.run(until=SIM_TIME)
    
    print(f"\nExperiment Results:")
    print(f"Total Deliveries: {met.completed_deliveries}")
    print(f"Separation Violations: {met.total_violations}")