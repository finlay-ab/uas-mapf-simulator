import numpy as np

class MAPFPolicy:
    # abstract base class for MAPF policies
    def plan_paths(self, name, pos, target, spatial_manager):
        raise NotImplementedError("This method should be overridden by a subclasses")
    