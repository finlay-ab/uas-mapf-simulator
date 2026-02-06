import unittest
import numpy as np
from src.environment import SpatialManager
from src.metrics import Metrics

class TestSafety(unittest.TestCase):

    def test_min_separation_logic(self):
        # set safety radius to 5 meters
        met = Metrics()
        sm = SpatialManager(safety_radius=5.0)

        # place two drones 3 meters apart
        pos1 = np.array([10.0, 10.0])
        pos2 = np.array([10.0, 13.0])

        # update pos
        sm.update("UAV_0", pos1, np.array([0,0]), "EN_ROUTE")
        sm.update("UAV_1", pos2, np.array([0,0]), "EN_ROUTE")

        # check for conflict
        sm.check_conflicts(met)

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

        sm.update("UAV_0", pos1, np.array([0,0]), "EN_ROUTE")
        sm.update("UAV_1", pos2, np.array([0,0]), "EN_ROUTE")

        sm.check_conflicts(met)
        stats = met.get_summary_statistics()

        # fail if it records a violation when it shouldnt
        self.assertEqual(stats["total violations"], 0, "FAIL: phantom violation recorded when drones were safe")

if __name__ == "__main__":
    unittest.main()