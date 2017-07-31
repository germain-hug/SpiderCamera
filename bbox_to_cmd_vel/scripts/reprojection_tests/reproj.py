import cv2
import math
import time
import numpy as np
import numpy.matlib as nplib
import functools
from scipy.ndimage.interpolation import map_coordinates as interp2
from scipy.interpolate import interp2d

"""
=========================================================
 Equirectangular to Stereographic Projection, as in :
http://mathworld.wolfram.com/StereographicProjection.html
=========================================================
"""

def compute_maps(w, h, dist, lambda_0, phi_1):

    # --- Useful constants ---
    xGrid, yGrid = np.meshgrid(range(1,w+1),range(1,h+1))
    rads = 2*math.pi/w
    z = w / dist

    # Convert to radians
    lambda_0 = lambda_0*rads
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
    lon = lambda_0 + np.asarray(map(longitude, xGrid.flatten(), yGrid.flatten())).reshape(xGrid.shape)

    # --- Compute Sampling maps ---
    x_map = w/2.0 + lon/rads
    y_map = h/2.0 - lat/rads

    return x_map.astype('float32'), y_map.astype('float32')


if __name__ == '__main__':

    im = cv2.imread('equirect_stream.png')
    h, w, _ = im.shape
    print("Computing projection maps...")
    x_map, y_map = compute_maps(w, h, -1.5, 0, 0)
    im2 = cv2.remap(im, x_map, y_map, cv2.INTER_CUBIC)
    cv2.imshow('Equirectangular image', im2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    #for i in range(-500,500):
    #    im2 = cv2.remap(np.roll(im,i,1), x_map, y_map, cv2.INTER_CUBIC)
    #    cv2.imshow('Equirectangular image', im2)
    #    cv2.waitKey(1)
