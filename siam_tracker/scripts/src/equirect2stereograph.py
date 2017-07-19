import cv2
import math
import time
import numpy as np
import numpy.matlib as nplib
import os.path
import functools
from scipy.ndimage.interpolation import map_coordinates as interp2
from scipy.interpolate import interp2d

"""
=========================================================
 Equirectangular to Stereographic Projection, as in :
http://mathworld.wolfram.com/StereographicProjection.html
=========================================================
"""

class equirect2stereograph:

    def __init__(self, lon, dist, im):
        h, w, _ = im.shape # Compute Projection maps
        self.x_map, self.y_map = self.compute_maps(w, h, dist, lon)

    def compute_maps(self, w, h, dist, z_rot):

        # --- Check if maps have already been computed --- 
        if(os.path.isfile('x_map.dat') & os.path.isfile('y_map.dat')):
            print("[INFO]: Projection Maps found : Loading...")
            x_map = np.fromfile('x_map.dat', dtype=float).reshape(h,w)
            y_map = np.fromfile('y_map.dat', dtype=float).reshape(h,w)

        else:
            print("[INFO]: Computing Projection Maps...")
            xGrid, yGrid = np.meshgrid(range(1, w + 1), range(1, h + 1))
            rads = 2 * math.pi / w
            z = w / dist

            # --- Define operators as lambda functions ---
            d = lambda x, y: x - y / 2.0
            a = lambda x, y: math.atan2(d(y, h), d(x, w))
            rho = lambda x, y: np.sqrt(d(x, w) ** 2 + d(y, h) ** 2)
            c = lambda x, y: 2 * math.atan(rho(x, y) / z)

            # --- Cartesian to Polar coordinates ---
            lat = np.asarray(map(c, xGrid.flatten(), yGrid.flatten())).reshape(xGrid.shape)
            lon = np.asarray(map(a, xGrid.flatten(), yGrid.flatten())).reshape(xGrid.shape) - math.pi / 4.0

            # --- Ensure correct coordinate wrapping ---
            lat = np.mod(lat + math.pi, math.pi) - math.pi / 2.0
            lon = np.mod(lon + math.pi + z_rot, math.pi * 2.0) - math.pi

            # --- Compute Sampling maps ---
            x_map = w / 2.0 + lon / rads
            y_map = h / 2.0 - lat / rads

            # --- Save maps ----
            x_map.tofile('x_map.dat')
            y_map.tofile('y_map.dat')


        return x_map.astype('float32'), y_map.astype('float32')

    def project(self, im):
        return cv2.remap(im, self.x_map, self.y_map, cv2.INTER_CUBIC)
