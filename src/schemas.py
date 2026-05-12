from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np
from src.physics import Velocity, GlobalPosition, LocalPosition, GridPosition

class PolicyType(Enum):
    GREEDY = auto()
    ASTAR = auto()
    DSTAR = auto()
    OCCUPANCY_ASTAR = auto()
    DFS = auto()
    BFS = auto()

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


class JobStatus(Enum):
    PENDING = auto()
    IN_PROGRESS = auto()
    COMPLETED = auto()
    FAILED = auto()

@dataclass
class Job:
    id: str
    origin_airspace: str
    origin_depot: str
    destination_airspace: str
    target_pos: GlobalPosition
    status: JobStatus = JobStatus.PENDING
    job_creation_time: float = None
    job_start_time: float = None
    job_completion_time: float = None

# in order of severity 
class AirspaceType(Enum):
    OPEN = 0
    RESTRICTED = auto()
    PROHIBITED = auto()
    OBSTACLE = auto()

@dataclass(frozen=True)
class Depot:
    id: str
    grid_position: GridPosition

class WayPointType(Enum):
    EN_ROUTE = auto()
    TAKEOFF = auto()
    HANDOVER_OUT = auto()
    HANDOVER_IN = auto()
    DELIVERY = auto()
    LANDING = auto()

@dataclass()
class Waypoint:
    position: GlobalPosition
    type: WayPointType 
    airspace_id: str
    gate_id: str = None
    depot_id: str = None

@dataclass()
class Gate:
    id: str
    position: GridPosition
    airspace_id: str
    target_airspace_id: str
    target_gate_id: str
    capacity: int
    queue_positions: list[GridPosition] = field(default_factory=list)

@dataclass()
class UAV_SEGMENT:
    start_position: GlobalPosition
    end_position: GlobalPosition
    start_time: float
    end_time: float
    velocity: Velocity
    radius: float
    predicted_collisions: list = field(default_factory=list)
    predicted_violations: list = field(default_factory=list)