import unittest
import numpy as np
from src.physics import Velocity
from src.policies.greedy import GreedyPolicy
from src.policies.vo_wrapper import VOWrapper

class MockSpatialManager:
    def __init__(self, position_map, velocity_map):
        self.positions = {k: np.array(v, dtype=float) for k, v in position_map.items()}
        self.velocities = {k: v for k, v in velocity_map.items()}

class TestVOWrapper(unittest.TestCase):

    def test_perfect_head_on_collision(self):
        base_policy = GreedyPolicy(5.0)
        wrapper = VOWrapper(base_policy, safety_radius=2.5, active=True)
        
        # set up head on collision
        spatial_manager = MockSpatialManager(
            position_map={"me": [0.0, 0.0], "other": [5.0, 0.0]},
            velocity_map={"me": Velocity(5.0, 0.0), "other": Velocity(-5.0, 0.0)}
        )
        
        my_pos = np.array([0.0, 0.0])
        my_goal = np.array([10.0, 0.0])
        
        # get velocity
        command_velocity = wrapper.get_velocity("me", my_pos, my_goal, spatial_manager)
        
        # check for symmetry break
        self.assertGreater(abs(command_velocity.vy), 0.1, "FAIL: symmetry breaker failed on dead-on collision")

    def test_offset_head_on(self):
        base_policy = GreedyPolicy(5.0)
        wrapper = VOWrapper(base_policy, safety_radius=2.5, active=True)
        
        # set up offset
        spatial_manager = MockSpatialManager(
            position_map={"me": [0.0, 0.0], "other": [5.0, 1.0]},
            velocity_map={"me": Velocity(5.0, 0.0), "other": Velocity(-5.0, 0.0)}
        )
        
        my_pos = np.array([0.0, 0.0])
        my_goal = np.array([10.0, 0.0])
        
        # get velocity
        command_velocity = wrapper.get_velocity("me", my_pos, my_goal, spatial_manager)
        
        # check for swerve
        self.assertGreater(abs(command_velocity.vy), 0.1, "FAIL: failed to swerve with natural offset")

    def test_stationary_avoidance(self):
        # the broken drone scenario: obstacle is not moving
        base_policy = GreedyPolicy(5.0)
        wrapper = VOWrapper(base_policy, safety_radius=2.5, active=True)
        
        # set up stationary obstacle
        spatial_manager = MockSpatialManager(
            position_map={"me": [0.0, 0.0], "other": [5.0, 0.0]},
            velocity_map={"me": Velocity(5.0, 0.0), "other": Velocity(0.0, 0.0)}
        )
        
        my_pos = np.array([0.0, 0.0])
        my_goal = np.array([10.0, 0.0])
        
        # get velocity
        command_velocity = wrapper.get_velocity("me", my_pos, my_goal, spatial_manager)
        
        # check for avoidance
        self.assertGreater(abs(command_velocity.vy), 0.1, "FAIL: failed to avoid stationary obstacle")

if __name__ == "__main__":
    unittest.main()