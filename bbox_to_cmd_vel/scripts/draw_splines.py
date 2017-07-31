import matplotlib
import numpy as np
from scipy.special import binom

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ==========================================
# Based on Bezier Drawing Code by Juanlu001
# https://gist.github.com/Juanlu001/7284462
# ==========================================


class BezierBuilder(object):
    """Bezier curve interactive builder.
    """
    def __init__(self, control_polygon):
        """Constructor.
        Receives the initial control polygon of the curve.
        """
        self.control_polygon = control_polygon
        self.xp = list(control_polygon.get_xdata())
        self.yp = list(control_polygon.get_ydata())
        self.canvas = control_polygon.figure.canvas
        self.ax_main = control_polygon.get_axes()

        # Event handler for mouse clicking
        self.cid = self.canvas.mpl_connect('button_press_event', self)

        # Create Bezier curve
        line_bezier = Line2D([], [],
                             c=control_polygon.get_markeredgecolor())
        self.bezier_curve = self.ax_main.add_line(line_bezier)

    def __call__(self, event):
        # Ignore clicks outside axes
        if event.inaxes != self.control_polygon.axes:
            return

        # Add point
        self.xp.append(event.xdata)
        self.yp.append(event.ydata)
        self.control_polygon.set_data(self.xp, self.yp)

        # Rebuild Bezier curve and update canvas
        self.bezier_curve.set_data(*self._build_bezier())
        self._update_bezier()

    def _build_bezier(self):
        x, y = Bezier(list(zip(self.xp, self.yp))).T
        return x, y

    def _update_bezier(self):
        self.canvas.draw()

def Bernstein(n, k):
    """Bernstein polynomial.
    """
    coeff = binom(n, k)

    def _bpoly(x):
        return coeff * x ** k * (1 - x) ** (n - k)

    return _bpoly


def Bezier(points, num=200):
    """Build Bezier curve from points.
    """
    N = len(points)
    t = np.linspace(0, 1, num=num)
    curve = np.zeros((num, 2))
    for ii in range(N):
        curve += np.outer(Bernstein(N - 1, ii)(t), points[ii])
    return curve
