import unittest
import simpy
import numpy as np
from src.entities import UAV, UAVState
from src.metrics import Metrics
from src.environment.spatial import SpatialManager 
from src.physics import Velocity
from src.config import SimConfig

class TestStates(unittest.TestCase):

    def test_flight_state_sequence(self):
        env = simpy.Environment()
        met = Metrics()
        sm = SpatialManager(safety_radius=2.5) 
        job_queue = simpy.Store(env)
        
        # ensure the job is available
        job_queue.put({'id': 1, 'goal': np.array([5, 5])})
        
        class MockPolicy:
            def plan_path(self, start, goal, spatial_manager=None):
                return [np.array(start, dtype=float), np.array(goal, dtype=float)]

            def get_velocity(self, id, pos, goal, sm):
                return Velocity(1.0, 1.0)

        uav = UAV(env, "UAV_0", [0,0], sm, met, MockPolicy(), job_queue, SimConfig())
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

    def test_takeoff_waits_for_clearance(self):
        env = simpy.Environment()
        met = Metrics()
        sm = SpatialManager(safety_radius=2.5)
        job_queue = simpy.Store(env)
        job_queue.put({'id': 1, 'goal': np.array([5, 5])})

        class MockPolicy:
            def plan_path(self, start, goal, spatial_manager=None):
                return [np.array(start, dtype=float), np.array(goal, dtype=float)]

            def get_velocity(self, id, pos, goal, sm):
                return Velocity(1.0, 1.0)

        cfg = SimConfig(depot_operation_radius=5.0, depot_check_interval=0.5)
        uav = UAV(env, "UAV_0", [0, 0], sm, met, MockPolicy(), job_queue, cfg)

        # active traffic starts close to depot, then clears later
        sm.update("UAV_blocker", np.array([1.0, 0.0]), Velocity(0.0, 0.0), UAVState.EN_ROUTE)

        def clear_blocker_later():
            yield env.timeout(1.5)
            sm.update("UAV_blocker", np.array([100.0, 100.0]), Velocity(0.0, 0.0), UAVState.EN_ROUTE)

        env.process(clear_blocker_later())

        # still blocked during early sim time
        env.run(until=1.0)
        self.assertEqual(uav.state, UAVState.TAKEOFF)

        # after blocker clears + takeoff timeout, UAV should be airborne
        env.run(until=4.5)
        self.assertIn(uav.state, [UAVState.EN_ROUTE, UAVState.HOVER_WAIT, UAVState.DELIVERING, UAVState.RETURNING])