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