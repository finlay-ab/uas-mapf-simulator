import unittest
import numpy as np
from src.environment.spatial import SpatialManager
from src.metrics import Metrics
from src.entities import UAVState

class TestSafety(unittest.TestCase):

    def test_min_separation_logic(self):
        # set safety radius to 5 meters
        met = Metrics()
        sm = SpatialManager(safety_radius=5.0)

        # place two drones 3 meters apart
        pos1 = np.array([10.0, 10.0])
        pos2 = np.array([10.0, 13.0])

        # update pos
        sm.update("UAV_0", pos1, np.array([0,0]), UAVState.EN_ROUTE)
        sm.update("UAV_1", pos2, np.array([0,0]), UAVState.EN_ROUTE)

        # check for conflict
        sm.check_conflicts(met, 0.0)

        # get metric
        stats = met.get_summary_statistics()

        # fail if violation wasn't counted
        self.assertEqual(stats["total violations"], 1, "FAIL: separation violation was not recorded")
        
        # fail if min separation is wrong
        self.assertEqual(stats["min separation (m)"], 3.0, f"FAIL: min separation math wrong! got {stats['min separation (m)']}")

    def test_no_violation_when_safe(self):
        met = Metrics()
        sm = SpatialManager(safety_radius=5.0)

        # drones 10 m apart (safe)
        pos1 = np.array([0.0, 0.0])
        pos2 = np.array([10.0, 0.0])

        sm.update("UAV_0", pos1, np.array([0,0]), UAVState.EN_ROUTE)
        sm.update("UAV_1", pos2, np.array([0,0]), UAVState.EN_ROUTE)

        sm.check_conflicts(met, 0.0)
        stats = met.get_summary_statistics()

        # fail if it records a violation when it shouldnt
        self.assertEqual(stats["total violations"], 0, "FAIL: phantom violation recorded when drones were safe")

    def test_ground_zone_reservation(self):
        sm = SpatialManager(safety_radius=5.0)
        depot = np.array([10.0, 10.0])

        # first UAV reserves zone
        self.assertTrue(sm.reserve_ground_zone("UAV_0", depot))
        self.assertFalse(sm.reserve_ground_zone("UAV_1", depot))

        # once released
        sm.release_ground_zone("UAV_0")
        self.assertTrue(sm.reserve_ground_zone("UAV_1", depot))

    def test_takeoff_landing_clearance_ignores_depot_internal_states(self):
        sm = SpatialManager(safety_radius=5.0)
        depot = np.array([0.0, 0.0])

        # drones in depot are ignored
        sm.update("idle", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.IDLE_DEPOT)
        sm.update("takeoff", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.TAKEOFF)
        sm.update("landing", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.LANDING)
        self.assertTrue(sm.is_takeoff_landing_clear("UAV_0", depot, clearance_radius=5.0))

        # active drone blocks
        sm.update("active", np.array([1.0, 0.0]), np.array([0.0, 0.0]), UAVState.EN_ROUTE)
        self.assertFalse(sm.is_takeoff_landing_clear("UAV_0", depot, clearance_radius=5.0))

    def test_idle_depot_drones_do_not_count_as_conflicts(self):
        met = Metrics()
        sm = SpatialManager(safety_radius=5.0)

        # test inside drones
        sm.update("UAV_0", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.IDLE_DEPOT)
        sm.update("UAV_1", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.IDLE_DEPOT)

        sm.check_conflicts(met, 0.0)
        stats = met.get_summary_statistics()

        self.assertEqual(stats["total violations"], 0)
        self.assertEqual(stats["min separation (m)"], "N/A")

    def test_takeoff_and_landing_do_not_count_as_ground_conflicts(self):
        met = Metrics()
        sm = SpatialManager(safety_radius=5.0)

        sm.update("UAV_0", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.TAKEOFF)
        sm.update("UAV_1", np.array([0.0, 0.0]), np.array([0.0, 0.0]), UAVState.LANDING)

        sm.check_conflicts(met, 0.0)
        stats = met.get_summary_statistics()

        self.assertEqual(stats["total violations"], 0)
        self.assertEqual(stats["min separation (m)"], "N/A")

if __name__ == "__main__":
    unittest.main()