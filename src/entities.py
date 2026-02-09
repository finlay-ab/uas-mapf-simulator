from enum import auto, Enum 
import numpy as np
import simpy
import logging
from physics import Velocity

# get log
log = logging.getLogger("UAS_Sim")

# enum to track state for metrics
class UAVState(Enum):
    IDLE_DEPOT = auto()
    TAKEOFF = auto()
    EN_ROUTE = auto()
    HOVER_WAIT = auto()
    DELIVERING = auto()
    RETURNING = auto()
    LANDING = auto()

# UAV entity which stores state and jobs
class UAV:
    def __init__(self, env, id, depot_pos, spartial_manager, metrics, policy, job_queue, dt=0.5):
        # set vars
        self.env = env
        self.uav_id = id
        self.depot = np.array(depot_pos, dtype=float)
        self.pos = np.array(depot_pos, dtype=float)
        self.vel = Velocity(0.0, 0.0)

        self.sm = spartial_manager
        self.metrics = metrics
        self.policy = policy
        self.job_queue = job_queue
        self.dt = dt
        
        # start at depot
        self.state = UAVState.IDLE_DEPOT
        self.current_job = None 
        
        # start process
        self.action = env.process(self.run())

    def run(self):
        # UAV loop
        while True:
            # idle at depot (waiting for job)
            self.state = UAVState.IDLE_DEPOT
            self.current_job = yield self.job_queue.get()
            target = np.array(self.current_job['goal'], dtype=float)
            
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} assigned to job {self.current_job['id']}")

            # takeoff phase (2s timeout to simulate takeoff)
            self.state = UAVState.TAKEOFF
            yield self.env.timeout(2.0) 

            # en_route to target
            start_delivery = self.env.now
            self.state = UAVState.EN_ROUTE
            yield from self.execute_flight(target)
            
            # record delivery time 
            self.metrics.record_delivery_phase(self.env.now - start_delivery)

            # drop off package (3s timeout to simulate delivery)
            self.state = UAVState.DELIVERING
            yield self.env.timeout(3.0) 
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} completed job {self.current_job['id']}")

            # return to depot
            start_return = self.env.now
            self.state = UAVState.RETURNING
            yield from self.execute_flight(self.depot)
            self.metrics.record_return_phase(self.env.now - start_return)
            
            # landing phase (2s timeout to simulate landing)
            self.state = UAVState.LANDING
            yield self.env.timeout(2.0)
            log.info(f"[{self.env.now:4.1f}] {self.uav_id} recovered at depot.")

            # clear job
            self.current_job = None

    def execute_flight(self, destination):
        # while not within 0.5 units of destination
        while np.linalg.norm(destination - self.pos) > 0.5:
            # follow policy to get velocity
            new_vel = self.policy.get_velocity(self.uav_id, self.pos, destination, self.sm)
            
            # if velocity is near zero, hover
            if new_vel.magnitude() < 0.1:
                self.state = UAVState.HOVER_WAIT
            else:
                # if returning to depot 
                if np.array_equal(destination, self.depot):
                    self.state = UAVState.RETURNING
                else:
                    self.state = UAVState.EN_ROUTE

            self.vel = new_vel
            
            # update pos
            self.pos += self.vel.as_array() * self.dt
            self.sm.update(self.uav_id, self.pos, self.vel.as_array(), self.state)
            
            # collect metrics
            self.metrics.record_path(self.uav_id, self.pos)
            
            yield self.env.timeout(self.dt)
        
        # arrived set vel to zero
        self.vel = Velocity(0.0, 0.0)
        self.sm.update(self.uav_id, self.pos, self.vel.as_array(), self.state)