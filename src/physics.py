import numpy as np

# velocity object to handle uav movement
class Velocity:
    def __init__(self, vx=0.0, vy=0.0):
        # store components as float
        self.vx = float(vx)
        self.vy = float(vy)

    def as_array(self):
        # return as numpy array for math
        return np.array([self.vx, self.vy])

    def magnitude(self):
        # return the speed (scalar)
        return np.linalg.norm(self.as_array())

    def limit(self, max_speed):
        # cap the velocity at max speed
        speed = self.magnitude()
        if speed > max_speed:
            scale = max_speed / speed
            self.vx *= scale
            self.vy *= scale
        return self

    def __repr__(self):
        # pretty print for logs
        return f"Vel({self.vx:.2f}, {self.vy:.2f})"

class GlobalPosition:
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def as_array(self):
        return np.array([self.x, self.y])

    def __repr__(self):
        return f"GlobalPos({self.x:.2f}, {self.y:.2f})"
    
    def __eq__(self, other):
        if not isinstance(other, GlobalPosition):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class LocalPosition:
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def as_array(self):
        return np.array([self.x, self.y])

    def __repr__(self):
        return f"LocalPos({self.x:.2f}, {self.y:.2f})"

    def __eq__(self, other):
        if not isinstance(other, LocalPosition):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class GridPosition:
    def __init__(self, gx=0, gy=0):
        self.gx = int(gx)
        self.gy = int(gy)

    def as_tuple(self):
        return (self.gx, self.gy)

    def __repr__(self):
        return f"GridPos({self.gx}, {self.gy})"

    def __eq__(self, other):
        if not isinstance(other, GridPosition):
            return NotImplemented
        return self.gx == other.gx and self.gy == other.gy
    
    def __hash__(self):
        return hash((self.gx, self.gy))