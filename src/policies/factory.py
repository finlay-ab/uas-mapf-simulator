from .greedy import GreedyPolicy
from .vo_wrapper import VOWrapper

def create_policy(config):
    # greedy as a base policy
    base = GreedyPolicy(config.max_speed)
    
    # wrap policy in VO and toggle it 
    return VOWrapper(base, config.safety_radius, active=config.use_vo)