import unittest

from src.entities import UAVState


class TestStates(unittest.TestCase):

    def test_expected_uav_states_exist(self):
        expected = [
            "IDLE_DEPOT",
            "TAKEOFF",
            "LANDING",
            "EN_ROUTE",
            "HOVER_WAIT",
            "DELIVERING",
            "RETURNING",
            "EMERGENCY",
            "BLOCKED",
            "HANDOVER",
        ]

        for name in expected:
            self.assertTrue(hasattr(UAVState, name), f"FAIL: missing UAV state {name}")

    def test_state_values_are_unique(self):
        values = [state.value for state in UAVState]
        self.assertEqual(len(values), len(set(values)), "FAIL: duplicate UAVState enum values")

    def test_idle_and_en_route_are_different(self):
        self.assertNotEqual(UAVState.IDLE_DEPOT, UAVState.EN_ROUTE, "FAIL: different states should not match")


if __name__ == "__main__":
    unittest.main()
