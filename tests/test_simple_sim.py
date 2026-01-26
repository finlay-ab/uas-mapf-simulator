
import simpy
import random
from simple_sim import Metrics, drone

def test_single_job_completes():
    random.seed(42) 
    env = simpy.Environment()
    hub = simpy.Resource(env, capacity=1)
    job_queue = simpy.Store(env)
    metrics = Metrics()

    # give constant flight time for test
    import simple_sim as s
    s.FLIGHT_TIME = (10, 10)

    # start one drone
    env.process(drone(env, "D1", hub, job_queue, metrics))

    # helper to push job to queue
    def push_one_job(env, q):
        yield env.timeout(1)
        yield q.put("JOB-1")
    
    # push 1 job at t=1
    env.process(push_one_job(env, job_queue))

    # allow for delivery + return + charge
    env.run(until=1 + 10 + 10 + 15 + 1)

    assert metrics.jobs_created == 1
    assert metrics.completed_deliveries == 1
    assert metrics.pad_charges == 1
