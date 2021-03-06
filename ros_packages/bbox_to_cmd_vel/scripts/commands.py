import numpy as np
from design import cart_to_spher

def update_motor_rel(x, y, z, m):
    # Returns relative displacement for motor commands (spher. coordinates)
    delta = []
    for i in range(0,len(m)):
        # Old Spherical Coordinates
        prev = cart_to_spher(m[i][0], m[i][1], m[i][2])
        # New Spherical Coordinates
        new = cart_to_spher(m[i][0] - x, m[i][1] - y, m[i][2] - z)
        # Relative Displacement
        delta.append(tuple(np.subtract(new,prev)))
    return delta
