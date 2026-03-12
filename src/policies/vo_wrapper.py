import numpy as np
from src.physics import Velocity 
from .base import PolicyWrapper

class VOCalculator:
    def __init__(self, safety_radius):
        self.radius = safety_radius

    def is_collision_course(self, va_cand, pos_a, pos_b, apex):
        # get pos
        relative_pos = pos_b - pos_a
        distance = np.linalg.norm(relative_pos)
        
        # if almost touching
        if distance < (2 * self.radius) * 0.98: 
            return True
            
        # danger cone
        phi = np.arcsin(min(1.0, (2 * self.radius) / distance))
        alpha = np.arctan2(relative_pos[1], relative_pos[0])
        
        # get relitive vel
        relative_vel = va_cand - apex
        theta = np.arctan2(relative_vel[1], relative_vel[0])
        
        # direction of collision
        if np.dot(relative_vel, relative_pos) < 0:
            return False

        # check if angle is within cone
        angle_diff = (theta - alpha + np.pi) % (2 * np.pi) - np.pi
        return abs(angle_diff) < phi

class VOWrapper(PolicyWrapper):
    def __init__(self, base_policy, safety_radius, active=True):
        super().__init__(base_policy, active=active)
        self.vo_calc = VOCalculator(safety_radius)
        self.max_speed = getattr(base_policy, 'max_speed', 5.0)

    def get_velocity(self, name, pos, target, spatial_manager):
        # get pref vel
        preferred_velocity_obj = self.base_policy.get_velocity(name, pos, target, spatial_manager)
        if not self.active: return preferred_velocity_obj

        # get other drones
        others_positions = spatial_manager.positions
        others_velocities = spatial_manager.velocities
        my_velocity = others_velocities.get(name, Velocity(0.0, 0.0)).as_array()

        # current safty status
        collision_likely = False
        for other_id, other_pos in others_positions.items():
            if other_id == name: continue
            
            # rvo apex
            apex = (my_velocity + others_velocities[other_id].as_array()) / 2
            if self.vo_calc.is_collision_course(preferred_velocity_obj.as_array(), pos, other_pos, apex):
                collision_likely = True
                break
        
        if not collision_likely: return preferred_velocity_obj

        # search for safe option
        diff = target - pos
        base_theta = np.arctan2(diff[1], diff[0])

        # sample circle starting left to right to avoid head on deadlock
        test_angles = [
            -0.4, 0.4,   # 23 degree dodges
            -0.8, 0.8,   # 45 degree dodges
            -1.57, 1.57, # 90 degree side-steps
            -2.2, 2.2,   # Turning away
            3.14         # Full reversal
        ]

        # avoid collision by changing angle and speed
        for speed_factor in [1.0, 0.7, 0.4]:
            velocity_magnitude = self.max_speed * speed_factor
            for angle in test_angles:
                test_theta = base_theta + angle
                candidate_velocity = np.array([np.cos(test_theta), np.sin(test_theta)]) * velocity_magnitude
                
                is_safe = True
                for other_id, other_pos in others_positions.items():
                    if other_id == name: continue
                    
                    # check new rvo apex
                    apex = (my_velocity + others_velocities[other_id].as_array()) / 2
                    if self.vo_calc.is_collision_course(candidate_velocity, pos, other_pos, apex):
                        is_safe = False
                        break
                
                if is_safe:
                    return Velocity(candidate_velocity[0], candidate_velocity[1])

        # if no safe option  stop
        return Velocity(0.0, 0.0)