import simpy
import random 
import logging

# config
SEED = 42
SIM_TIME = 200

NUM_DRONES = 3
NUM_PADS = 2

FLIGHT_TIME = (10, 20)
CHARGE_TIME = 15
REQUEST_INTERVAL = (5, 15)

# logging setup
logging.basicConfig(level=logging.INFO, format="%(message)s")
log = logging.getLogger("simple_sim")

# metrics
class Metrics:
    def __init__(self):
        self.completed_deliveries = 0
        self.jobs_created = 0
        self.pad_charges = 0

    def record_job_created(self):
        self.jobs_created += 1

    def record_delivery_completed(self):
        self.completed_deliveries += 1

    def record_charge(self):
        self.pad_charges += 1

# drone process
def drone(env, name, hub, job_queue, metrics: Metrics):
    global completed_deliveries

    while True:
        # wait for job
        job = yield job_queue.get()
        log.info(f"[{env.now:4}] {name} assigned job {job}")

        # 'fly to' delivery 
        flight_time = random.randint(*FLIGHT_TIME)
        log.info(f"[{env.now:4}] {name} flying to delivery ({flight_time})")
        yield env.timeout(flight_time)

        # deliver
        log.info(f"[{env.now:4}] {name} delivered job {job}")
        metrics.record_delivery_completed()

        # Return to hub
        log.info(f"[{env.now:4}] {name} returning to hub")
        yield env.timeout(flight_time)

        #request charging
        with hub.request() as req:
            yield req 

        log.info(f"[{env.now:4}] {name} charging")
        yield env.timeout(CHARGE_TIME)
        log.info(f"[{env.now:4}] {name} finished charging")
        metrics.record_charge()

# delivery generator
def delivery_generator(env, job_queue, metrics: Metrics):
    job_id = 0
    while True:
        interarrival = random.randint(*REQUEST_INTERVAL)
        yield env.timeout(interarrival)

        job_id += 1
        metrics.record_job_created()
        log.info(f"[{env.now:4}] new delivery request {job_id}")
        yield job_queue.put(job_id)

# main
def main():
    random.seed(SEED)

    env = simpy.Environment()
    metrics = Metrics()

    # hub with limited capacity for charging
    hub = simpy.Resource(env, capacity=NUM_PADS)

    # job queue
    job_queue = simpy.Store(env)

    # start creating deliveries
    env.process(delivery_generator(env, job_queue, metrics))

    # start drones
    for i in range(NUM_DRONES):
        env.process(drone(env, f"drone {i}", hub, job_queue, metrics))

    # run simulation
    env.run(until=SIM_TIME)
    print("\nsimulation finished")
    print(f"completed deliveries: {metrics.completed_deliveries}")
    print(f"jobs created: {metrics.jobs_created}")
    print(f"pad charges: {metrics.pad_charges}")

if __name__ == "__main__":
    main()
