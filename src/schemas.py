from enum import Enum, auto

class PolicyType(Enum):
    GREEDY = auto()
    ASTAR = auto()
    DSTAR = auto()
    DYNAMIC_ASTAR = auto()

class WrapperType(Enum):
    NONE = auto()
    VO = auto()

class PathRecoveryStrategy(Enum):
    RETURN_TO_DEVIATION = auto()
    RETURN_TO_CLOSEST_WP = auto()
    RETURN_TO_NEXT_WP = auto()

class PathRecoveryAction(Enum):
    RETURN_TO_PATH = auto()
    REPLAN = auto()
