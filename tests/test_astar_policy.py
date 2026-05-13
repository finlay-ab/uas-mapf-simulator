import unittest
import json
import tempfile
from pathlib import Path

import numpy as np

from src.environment.map import AirspaceType, GridMap
from src.policies.astar import AStarPolicy
from src.helpers import to_array

class TestAStarPolicy(unittest.TestCase):
    def _new_map(self, width=10, height=10):
        map_data = {
            "id": "test_map",
            "width": width,
            "height": height,
            "resolution": 1.0,
            "potential_field": False,
            "obstacles": [],
            "restricted_areas": [],
            "depots": [{"id": "depot1", "x": 1.0, "y": 1.0}],
            "gates": [],
        }
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
        try:
            json.dump(map_data, tmp)
            tmp.close()
            grid_map = GridMap(tmp.name)
        finally:
            Path(tmp.name).unlink(missing_ok=True)
        grid_map.grid[:, :] = AirspaceType.OPEN.value
        grid_map.weighted_grid[:, :] = 1.0
        return grid_map

    def test_connectivity_validation(self):
        grid_map = self._new_map(10, 10)
        with self.assertRaises(ValueError):
            AStarPolicy(grid_map, connectivity=6)

    def test_restricted_cells_never_in_path(self):
        grid_map = self._new_map(8, 8)
    
        grid_map.add_restricted_area(3.0, 4.0, 3.0, 4.0)

        policy = AStarPolicy(grid_map, drone_radius=0.0, connectivity=4)

        start = np.array([1.0, 3.0], dtype=float)
        goal = np.array([6.0, 3.0], dtype=float)

        path = policy.plan_path(start, goal)

        self.assertGreaterEqual(len(path), 2)
        for waypoint in path:
            wp = to_array(waypoint)
            airspace = grid_map.evaluate_footprint(float(wp[0]), float(wp[1]), 0.0)
            self.assertNotEqual(airspace, AirspaceType.RESTRICTED.value)

    def test_eight_connectivity_can_use_diagonal_steps(self):
        grid_map = self._new_map(10, 10)
        policy = AStarPolicy(grid_map, drone_radius=0.0, connectivity=8)

        start = np.array([1.0, 1.0], dtype=float)
        goal = np.array([5.0, 5.0], dtype=float)
        path = policy.plan_path(start, goal)

        self.assertGreaterEqual(len(path), 2)

        has_diagonal_step = False
        for i in range(len(path) - 1):
            a = to_array(path[i])
            b = to_array(path[i + 1])
            dx = abs(int(b[0]) - int(a[0]))
            dy = abs(int(b[1]) - int(a[1]))
            if dx == 1 and dy == 1:
                has_diagonal_step = True
                break

        self.assertTrue(has_diagonal_step)


if __name__ == "__main__":
    unittest.main()
