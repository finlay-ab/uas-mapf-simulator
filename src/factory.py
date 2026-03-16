from src.policies import *
from src.wrappers import *

def create_policy(config):
    # greedy as a base policy
    base = GreedyPolicy(config.max_speed)
    
    # wrap policy in VO and toggle it 
    return VOWrapper(base, config.safety_radius, active=config.use_vo)