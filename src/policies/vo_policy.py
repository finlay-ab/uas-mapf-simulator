import numpy as np
from .base import MAPFPolicy
from physics import Velocity

class VOCalculator:
    def __init__(self, safety_radius):
        self.radius = safety_radius

    def is_collision_course(self, va_cand, pos_a, pos_b, vb):
        # get pos and vel
        rel_pos = pos_b - pos_a
        rel_vel = va_cand - vb
        
        dist = np.linalg.norm(rel_pos)
        if dist < 0.001: return True
        
        # calcu cone
        alpha = np.arctan2(rel_pos[1], rel_pos[0])
        combined_radius = 2 * self.radius
        if combined_radius >= dist:
            return True 
            
        phi = np.arcsin(combined_radius / dist)
        
        # angle or relitive vel
        theta = np.arctan2(rel_vel[1], rel_vel[0])
        
        # check if angle is within cone
        angle_diff = (theta - alpha + np.pi) % (2 * np.pi) - np.pi
        return abs(angle_diff) < phi

# greedy with VO
class VOPolicy(MAPFPolicy):
    def __init__(self, max_speed, safety_radius):
        self.max_speed = max_speed
        self.vo_calc = VOCalculator(safety_radius)

    def get_velocity(self, name, pos, target, spatial_manager):
        # get greedy 
        direction = target - pos
        dist = np.linalg.norm(direction)
        
        if dist < 0.5:
            return Velocity(0, 0)
            
        v_pref = (direction / dist) * self.max_speed
        
        # get other drones
        others_pos = spatial_manager.positions
        others_vel = spatial_manager.velocities
        
        # check if pref vel is safe
        collision_likely = False
        for other_id in others_pos:
            if other_id == name: continue
            
            if self.vo_calc.is_collision_course(v_pref, pos, others_pos[other_id], others_vel[other_id]):
                collision_likely = True
                break
        
        if not collision_likely:
            return Velocity(v_pref[0], v_pref[1]).limit(self.max_speed)
            
        # avoid collision by changing angle
        for angle in [0.4, -0.4, 0.8, -0.8, 1.2, -1.2]:
            c, s = np.cos(angle), np.sin(angle)
            v_cand = np.array([
                v_pref[0] * c - v_pref[1] * s,
                v_pref[0] * s + v_pref[1] * c
            ])
            
            safe_choice = True
            for other_id in others_pos:
                if other_id == name: continue
                if self.vo_calc.is_collision_course(v_cand, pos, others_pos[other_id], others_vel[other_id]):
                    safe_choice = False
                    break
            
            if safe_choice:
                return Velocity(v_cand[0], v_cand[1]).limit(self.max_speed)
                
        # fail safe: slow down
        return Velocity(v_pref[0] * 0.2, v_pref[1] * 0.2)