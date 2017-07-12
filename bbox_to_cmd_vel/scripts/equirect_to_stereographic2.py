import cv2
import math
import time
import numpy as np
import numpy.matlib as nplib
from scipy.ndimage.interpolation import map_coordinates as interp2
from scipy.interpolate import interp2d


def equirect2stereograph(im, cam_dist, world_rotation):
    h, w, _ = im.shape
    z = w / cam_dist
    rads = 2*math.pi/w

    d = lambda i,j: i-j/2
    r = lambda x,y: np.sqrt(d(x,w)**2+d(y,h)**2)
    rho = lambda x,y: r(x,y)/z
    theta = lambda x,y: 2*math.atan(rho(x,y))
    a = lambda x,y: math.atan2(d(y,h),d(x,w))

    pixX, pixY = np.meshgrid(range(1,w+1),range(1,h+1))
    # Calculate polar coordinates
    lat = np.asarray(map(theta,pixX.flatten(),pixY.flatten())).reshape(pixX.shape)
    lon = np.asarray(map(a,pixX.flatten(),pixY.flatten())).reshape(pixX.shape) - math.pi/4
    # Wrap the lat & lon coordinates using mod
    lat = np.mod(lat + math.pi,math.pi) - math.pi/2
    lon = np.mod(lon + math.pi + world_rotation,math.pi*2) - math.pi
    # Back to equirectangular for sampling
    xe = w/2.0 - (-lon/rads)
    ye = h/2.0 - (lat/rads)
    return xe.astype('float32'), ye.astype('float32')


if __name__ == '__main__':

    im = cv2.imread('equirect.jpg')
    xe, ye = equirect2stereograph(im, 5, 0)
    t = time.time()
    im2 = cv2.remap(im, xe, ye, cv2.INTER_CUBIC)
    print(time.time()-t)
    cv2.imshow('Equirectangular image', im2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
