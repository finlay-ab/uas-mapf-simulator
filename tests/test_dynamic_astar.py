import unittest
import numpy as np

from src.environment.map import GridMap
from src.policies.occupancy_astar import OccupancyAStarPolicy

 
class MockSpatialManager:
    def __init__(self): 
        self.positions = {} 
        self.velocities = {}
        self.states = {}
 
    def update(self, uav_id, pos, vel, state):
        self.positions[uav_id] = np.array(pos, dtype=float)
        self.velocities[uav_id] = np.array(vel, dtype=float) 


class TestOccupancyAStarPolicy(unittest.TestCase):
    def _new_map(self):
        # simple empty map for path tests
        return GridMap(width=400, height=400, resolution=1.0) 

    def test_occupancy_astar_without_spatial_manager(self): 
        # no dynamic objects should still plan a normal path
        grid_map = self._new_map()  
        policy = OccupancyAStarPolicy(    
            grid_map,  
            max_speed=10.0, 
            safety_radius=2.0, 
            connectivity=4,  
        )  
 
        start = np.array([10.0, 10.0])
        goal = np.array([100.0, 100.0])
 
        path = policy.plan_path(start, goal, spatial_manager=None)  
        # check we got a valid start to goal path         
        self.assertGreaterEqual(len(path), 2)  
        self.assertTrue(np.allclose(path[0], start)) 
        self.assertTrue(np.allclose(path[-1], goal)) 

    def test_occupancy_astar_avoids_other_uav(self):
        # put 2 uavs near the direct route so planner should detour
        grid_map = self._new_map()
        policy = OccupancyAStarPolicy(
            grid_map,
            max_speed=10.0,
            safety_radius=2.0,
            connectivity=4,
        )

        sm = MockSpatialManager()
        sm.positions["uav_1"] = np.array([50.0, 50.0])
        sm.positions["uav_2"] = np.array([55.0, 50.0])

        start = np.array([40.0, 50.0])
        goal = np.array([100.0, 50.0])

        path = policy.plan_path(start, goal, spatial_manager=sm)

        # path still needs to be valid
        self.assertGreaterEqual(len(path), 2)
        self.assertTrue(np.allclose(path[0], start))
        self.assertTrue(np.allclose(path[-1], goal))

        # dont pass too close to blocked uav center
        for waypoint in path:
            dist_to_blocked = np.linalg.norm(waypoint - np.array([50.0, 50.0]))
            self.assertTrue(dist_to_blocked > 1.0 or np.allclose(waypoint, start))

    def test_occupancy_astar_preserves_connectivity(self):
        # connectivity setting should still matter with dynamic blocking on
        grid_map = self._new_map()

        sm = MockSpatialManager()
        sm.positions["other_uav"] = np.array([200.0, 200.0])

        policy_4 = OccupancyAStarPolicy(
            grid_map,
            max_speed=10.0,
            safety_radius=1.0,
            connectivity=4,
        )

        start = np.array([10.0, 10.0])
        goal = np.array([390.0, 390.0])
        path_4 = policy_4.plan_path(start, goal, spatial_manager=sm)
        self.assertGreaterEqual(len(path_4), 2)

        policy_8 = OccupancyAStarPolicy(
            grid_map,
            max_speed=10.0,
            safety_radius=1.0,
            connectivity=8,
        )

        path_8 = policy_8.plan_path(start, goal, spatial_manager=sm)
        self.assertGreaterEqual(len(path_8), 2)
        # 8-connected should be same or shorter than 4-connected path
        self.assertLessEqual(len(path_8), len(path_4))

    def test_occupancy_astar_blocked_set_building(self):
        # directly test blocked-cell generation from spatial positions
        grid_map = self._new_map()
        policy = OccupancyAStarPolicy(
            grid_map,
            max_speed=10.0,
            safety_radius=2.0,
            connectivity=4,
        )

        sm = MockSpatialManager()
        sm.positions["uav_1"] = np.array([50.0, 50.0])
        sm.positions["uav_2"] = np.array([100.0, 100.0])

        blocked = policy._build_dynamic_blocked_set(sm)

        # should include center + nearby cells, and not random far cells
        self.assertGreater(len(blocked), 0)
        self.assertIn((50, 50), blocked)
        self.assertTrue((50, 51) in blocked or (51, 50) in blocked or (51, 51) in blocked)
        self.assertNotIn((0, 0), blocked)
        self.assertNotIn((399, 399), blocked)

    def test_occupancy_astar_empty_spatial_manager(self):
        # empty spatial manager should behave like normal astar
        grid_map = self._new_map()
        policy = OccupancyAStarPolicy(
            grid_map,
            max_speed=10.0,
            safety_radius=2.0,
        )

        sm = MockSpatialManager()

        start = np.array([10.0, 10.0])
        goal = np.array([100.0, 100.0])

        path = policy.plan_path(start, goal, spatial_manager=sm)
        self.assertGreaterEqual(len(path), 2)


if __name__ == "__main__":
    unittest.main()
