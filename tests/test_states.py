import unittest
import simpy
import numpy as np
from src.entities import UAV, UAVState
from src.metrics import Metrics
from src.environment import SpatialManager 

class TestStates(unittest.TestCase):

    def test_flight_state_sequence(self):
        env = simpy.Environment()
        met = Metrics()
        sm = SpatialManager(safety_radius=2.5) 
        job_queue = simpy.Store(env)
        
        # ensure the job is available
        job_queue.put({'id': 1, 'goal': np.array([5, 5])})
        
        class MockPolicy:
            def get_velocity(self, id, pos, goal, sm):
                return np.array([1.0, 1.0])

        uav = UAV(env, "UAV_0", [0,0], sm, met, MockPolicy(), job_queue)
        state_history = []

        def observer():
            while True:
                # get current state
                if not state_history or uav.state != state_history[-1]:
                    state_history.append(uav.state)
                yield env.timeout(0.01) 

        env.process(observer())
        
        # run for progression
        env.run(until=50)

        #verify the expected sequence exists
        expected_start = [UAVState.IDLE_DEPOT, UAVState.TAKEOFF, UAVState.EN_ROUTE]
        self.assertEqual(state_history[:3], expected_start, f"Sequence mismatch. Got: {state_history}")

        # verify chronological order
        if UAVState.LANDING in state_history:
            self.assertLess(
                state_history.index(UAVState.TAKEOFF), 
                state_history.index(UAVState.LANDING),
                "UAV reached LANDING state before TAKEOFF state."
            )