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

    def compute_maps(self, w, h, dist):

        # --- Check if maps have already been computed ---
        if(os.path.isfile('x_map.dat') & os.path.isfile('y_map.dat')):
            print("[INFO]: Projection Maps found : Loading...")
            x_map = np.fromfile('x_map.dat', dtype=float).reshape(h,w)
            y_map = np.fromfile('y_map.dat', dtype=float).reshape(h,w)

        else:
            print("[INFO]: Computing Projection Maps...")

            x_map = numpy.zeros(shape=(h,w,360))
            y_map = numpy.zeros(shape=(h,w,360))

            for phi_1 in range(0,360): # All latitude offsets

                xGrid, yGrid = np.meshgrid(range(1,w+1),range(1,h+1))
                rads = 2*math.pi/w
                z = w / dist

                # --- Convert to radians ---
                phi_1 = phi_1*rads
                s_p = math.sin(phi_1)
                c_p = math.cos(phi_1)

                # --- Define operators as lambda functions ---
                d = lambda x,y: x-y/2.0
                a = lambda x,y: math.atan2(d(y,h),d(x,w))
                rho = lambda x,y: np.sqrt(d(x,w)**2+d(y,h)**2)

                c = lambda x,y: 2*math.atan(rho(x,y)/z)
                s_c = lambda x,y: math.sin(c(x,y))
                c_c = lambda x,y: math.cos(c(x,y))

                longitude = lambda x,y: math.atan2(d(x,w)*s_c(x,y),rho(x,y)*c_p*c_c(x,y)- d(y,h)*s_p*s_c(x,y))
                latitude = lambda x,y: math.asin(c_c(x,y)*s_p + d(y,h)*s_c(x,y)*c_p/(rho(x,y)+0.00000001))

                # --- Cartesian to Polar coordinates ---
                lat = np.asarray(map(latitude, xGrid.flatten(), yGrid.flatten())).reshape(xGrid.shape)
                lon = np.asarray(map(longitude, xGrid.flatten(), yGrid.flatten())).reshape(xGrid.shape)

                # --- Compute Sampling maps ---
                x_map[:,:,phi_1] = w/2.0 + lon/rads
                y_map[:,:,phi_1] = h/2.0 - lat/rads

            # --- Save maps ----
            x_map.tofile('x_map.dat')
            y_map.tofile('y_map.dat')


        return x_map.astype('float32'), y_map.astype('float32')

    def project(self, im):
        return cv2.remap(im, self.x_map, self.y_map, cv2.INTER_CUBIC)
