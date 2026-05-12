import numpy as np

# get xy array from position objects
def get_xy(position):
    # global/ local
    if hasattr(position, 'x') and hasattr(position, 'y'):
        return float(position.x), float(position.y)

    # grid pos
    if hasattr(position, 'gx') and hasattr(position, 'gy'):
        return float(position.gx), float(position.gy)
    
    # np arr
    if isinstance(position, np.ndarray):
        if len(position) >= 2:
            return float(position[0]), float(position[1])
        raise ValueError(f"array must have at least 2 elements: {position}")
    
    # tuple/ list
    if isinstance(position, (tuple, list)):
        if len(position) >= 2:
            return float(position[0]), float(position[1])
        raise ValueError(f"tuple/ list must have at least 2 elements: {position}") 
    
    raise TypeError(f"unsupported position type: {type(position)}")

# return position as a float numpy array [x, y]
def to_array(position):
    # if has arr method 
    if hasattr(position, 'as_array'):
        return np.asarray(position.as_array(), dtype=float)

    # else get x,y
    x, y = get_xy(position)
    return np.array([x, y], dtype=float)

# gets the euclidean distance between 2 points
def distance_between(pos1, pos2):
    arr1 = to_array(pos1)
    arr2 = to_array(pos2)
    return float(np.linalg.norm(arr2 - arr1))