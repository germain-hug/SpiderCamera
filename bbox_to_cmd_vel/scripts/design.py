import json
import math
from collections import namedtuple


def cart_to_spher(x,y,z):
    r = math.sqrt(x*x+y*y+z*z)
    if(x!=0):
        theta = math.atan(y/x)
    else:
        theta = 0.0

    if(r!=0):
        phi = math.acos(z/r)
    else:
        phi = 0.0

    return r, theta, phi

def compute_motor_coords(x, y, z):
    r, t, p = cart_to_spher(x, y, z)
    return [x, y, z, r, t, p]

def __init__():
    global m # Motors
    # (m_i) : (x_i, y_i, z_i, rho_i, theta_i, phi_i)
    # (x,y,z) : world coordinates of camera

    # ---- Parse from JSON file ----
    with open('design.json') as json_file:
        design = json.load(json_file)
    design = namedtuple('design', design.keys())(**design)

    # ----- Camera Initial world Coordinates ----
    x = design.width/2
    y = design.depth/2
    z = design.height - design.height_cam

    # ----- Motor Coordinates ----
    m = []
    m.append(compute_motor_coords(x, -y, z))
    m.append(compute_motor_coords(-x, -y, z))
    m.append(compute_motor_coords(-x, y, z))
    m.append(compute_motor_coords(x, y, z))
