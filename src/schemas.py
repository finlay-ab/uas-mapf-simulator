from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
from src.physics import Velocity, GlobalPosition, LocalPosition, GridPosition

class PolicyType(Enum):
    GREEDY = auto()
    ASTAR = auto()
    DSTAR = auto()
    OCCUPANCY_ASTAR = auto()

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

class PathRecoveryAction(Enum):
    RETURN_TO_PATH = auto()
    REPLAN = auto()


class JobStatus(Enum):
    PENDING = auto()
    IN_PROGRESS = auto()
    COMPLETED = auto()
    FAILED = auto()

@dataclass
class Job:
    id: int
    origin_airspace: str
    origin_depot: str
    destination_airspace: str
    target_pos: np.ndarray
    status: JobStatus = JobStatus.PENDING   
    job_creation_time: float = None
    job_start_time: float = None
    job_end_time: float = None

# in order of severity 
class AirspaceType(Enum):
    OPEN = 0
    RESTRICTED = auto()
    PROHIBITED = auto()
    OBSTACLE = auto()

@dataclass(frozen=True)
class Depot:
    id: int
    grid_position: GridPosition