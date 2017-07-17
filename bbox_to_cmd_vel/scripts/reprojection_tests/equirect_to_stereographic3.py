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

def rotate_xz(z, phi, x, y):
    return np.cos(np.deg2rad(phi))*x - np.sin(np.deg2rad(phi))*z, y, np.sin(np.deg2rad(phi))*x + np.cos(np.deg2rad(phi))*z

def rotate_yz(phi, x, y):
    return x, np.cos(np.deg2rad(phi))*y - np.sin(np.deg2rad(phi))*z, np.sin(np.deg2rad(phi))*y + np.cos(np.deg2rad(phi))*z

def compute_maps(w, h, dist):

    xGrid, yGrid = np.meshgrid(range(-w/2,w/2),range(-h/2,h/2))
    zGrid = np.full(xGrid.shape, w / dist)

    u = np.asarray(map(lambda x,z: 2*x/(1-z), xGrid, zGrid))
    v = np.asarray(map(lambda y,z: 2*y/(1-z), yGrid, zGrid))

    rho = np.asarray(map(lambda u,v: np.sqrt(u**2+v**2), u,v))
    theta = np.asarray(map(lambda u,v: math.atan2(u,v), u.flatten(),v.flatten())).reshape(u.shape)

    u = map(lambda r,t: -r*np.sin(t), rho, theta)
    v = map(lambda r,t: r*np.cos(t), rho, theta)

    x = map(lambda u,v: 4*u/(u**2+v**2+4), u, v)
    y = map(lambda u,v: 4*v/(u**2+v**2+4), u, v)
    z = map(lambda u,v: (u**2+v**2-4)/(u**2+v**2+4), u, v)

    x = np.asarray(map(lambda x, z: x/(-z), x, z))
    y = np.asarray(map(lambda y, z: y/(-z), y, z))

    print(x, y)

    return x.astype('float32'), y.astype('float32')


if __name__ == '__main__':

    im = cv2.imread('equirect.jpg')
    h, w, _ = im.shape
    print("Computing projection maps...")
    xe, ye = compute_maps(w, h, 40)
    print("done")

    t = time.time()
    im2 = cv2.remap(im, xe, ye, cv2.INTER_CUBIC)
    print(time.time()-t)
    cv2.imshow('Equirectangular image', im2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
