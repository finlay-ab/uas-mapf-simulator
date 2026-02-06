import unittest
import numpy as np
from src.policies.greedy import GreedyPolicy

class TestPhysics(unittest.TestCase):
    
    def test_speed_limit(self):
        max_speed = 5.0
        pol = GreedyPolicy(max_speed)
        
        pos = np.array([0.0, 0.0])
        goal = np.array([100.0, 100.0])
        
        # get velocity from policy
        vel = pol.get_velocity("UAV_0", pos, goal, None)
        
        # calculate actual speed
        speed = np.linalg.norm(vel)
        
        self.assertLessEqual(speed, max_speed, f"FAIL: drone exceeded speed limit! {speed} > {max_speed}")

    def test_zero_velocity_at_goal(self):
        # drone should stop when it arrives
        max_speed = 5.0
        pol = GreedyPolicy(max_speed)
        
        # pos and goal are exactly the same
        pos = np.array([50.0, 50.0])
        goal = np.array([50.0, 50.0])
        
        vel = pol.get_velocity("UAV_0", pos, goal, None)
        speed = np.linalg.norm(vel)
        
        # should stop
        self.assertEqual(speed, 0.0, "FAIL: drone did not stop at goal")

if __name__ == "__main__":
    unittest.main()