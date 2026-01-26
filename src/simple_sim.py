import simpy
import random 

# config
SEED = 42
SIM_TIME = 200

NUM_DRONES = 3
NUM_PADS = 2

FLIGHT_TIME = (10, 20)
CHARGE_TIME = 15
REQUEST_INTERVAL = (5, 15)

# metric 
completed_deliveries = 0

# drone process
def drone(env, name, hub, job_queue):
    global completed_deliveries

    while True:
        # wait for job
        job = yield job_queue.get()
        print(f"[{env.now:4}] {name} assigned job {job}")

        # 'fly to' delivery 
        flight_time = random.randint(*FLIGHT_TIME)
        print(f"[{env.now:4}] {name} flying to delivery ({flight_time})")
        yield env.timeout(flight_time)

        # deliver
        print(f"[{env.now:4}] {name} delivered job {job}")
        completed_deliveries += 1

        # Return to hub
        print(f"[{env.now:4}] {name} returning to hub")
        yield env.timeout(flight_time)

        #request charging
        with hub.request() as req:
            yield req 
                    
        print(f"[{env.now:4}] {name} charging")
        yield env.timeout(CHARGE_TIME)
        print(f"[{env.now:4}] {name} finished charging")

# delivery generator
def delivery_generator(env, job_queue):
    job_id = 0
    while True:
        interarrival = random.randint(*REQUEST_INTERVAL)
        yield env.timeout(interarrival)

        job_id += 1
        print(f"[{env.now:4}] new delivery request {job_id}")
        yield job_queue.put(job_id)

# main
def main():
    random.seed(SEED)

    env = simpy.Environment()

    # hub with limited capacity for charging
    hub = simpy.Resource(env, capacity=NUM_PADS)

    # job queue
    job_queue = simpy.Store(env)

    # start creating deliveries
    env.process(delivery_generator(env, job_queue))

    # start drones
    for i in range(NUM_DRONES):
        env.process(drone(env, f"drone {i}", hub, job_queue))

    # run simulation
    env.run(until=SIM_TIME)
    print("\n simulation finished")
    print(f"completed deliveries: {completed_deliveries}")


if __name__ == "__main__":
    main()
