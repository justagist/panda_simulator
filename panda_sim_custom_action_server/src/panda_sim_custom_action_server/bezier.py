# Copyright (c) 2011, Ian McMahon
# Modifications Copyright (c) 2014-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The Bezier library was  implemented as a class project in CIS515,
Fundamentals of Linear Algebra, taught by Professor Jean Gallier
in the summer of 2011 at the University of Pennsylvania. For an
excellent explanation of Cubic Bezier Curves, and the math
represented in this library, see
http://www.cis.upenn.edu/~cis515/proj1-12.pdf

~~~~~~~~~~~~~~~~~~~~~~~~ Bezier ~~~~~~~~~~~~~~~~~~~~~~~~
A library for computing Bezier Cubic Splines for an arbitrary
set of control points in R2, R3, up to RN space.

Cubic Segment:
C(t) = (1 - t)^3*b0 + 3(1 - t)*b1 + 3(1 - t)*t^2*b2 + t^3*b3

Bezier Spline of Cubic Segments:
B(t) = C_(i)(t-i+1), i-1 <= t <= i
where C0 continuity exists: C_(i)(1) = C_(i+1)(0)
where C1 continuity exists: C'_(i)(1) = C'_(i+1)(0)
and where C2 continuity exists: C"_(i)(1) = C"_(i+1)(0)

ex. usage:
import numpy
import bezier
points_array = numpy.array([[1, 2, 3], [4, 4, 4],
                            [6, 4, 6], [2, 5, 6],
                            [5, 6, 7]])
d_pts = bezier.de_boor_control_pts(points_array)
b_coeffs = bezier.bezier_coefficients(points_array, d_pts)
b_curve = bezier.bezier_curve(b_coeffs, 50)
#  plotting example
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')
#plot bezier curve
ax.plot(b_curve[:,0], b_curve[:,1], b_curve[:,2])
#plot specified points
ax.plot(points_array[:,0], points_array[:,1], points_array[:,2], 'g*')
ax.set_title("Cubic Bezier Spline")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend(["Bezier Curve", "Control Points"], loc=2)
plt.show()
"""
import numpy as np


def de_boor_control_pts(points_array, d0=None,
                        dN=None, natural=True):
    """
    Compute the de Boor control points for a given
    set for control points

    params:
        points_array: array of user-supplied control points
            numpy.array of size N by k
            N is the number of input control points
            k is the number of dimensions for each point

        d0: the first control point - None if "natural"
            numpy.array of size 1 by k

        dN: the last control point - None if "natural"
            numpy.array of size 1 by k

        natural: flag to signify natural start/end conditions
            bool

    returns:
        d_pts: array of de Boor control points
            numpy.array of size N+3 by k
    """
    # N+3 auxiliary points required to compute d_pts
    # dpts_(-1) = x_(0)
    # dpts_(N+1) = x_(N)
    # so it is only necessary to find N+1 pts, dpts_(0) to to dpts_(N)
    (rows, k) = np.shape(points_array)
    N = rows - 1  # minus 1 because list includes x_(0)
    # Compute A matrix
    if natural:
        if N > 2:
            A = np.zeros((N-1, N-1))
            A[np.ix_([0], [0, 1])] = [4, 1]
            A[np.ix_([N-2], [N-3, N-2])] = [1, 4]
        else:
            A = 4.0
    else:
        if N > 2:
            A = np.zeros((N-1, N-1))
            A[np.ix_([0], [0, 1])] = [3.5, 1]
            A[np.ix_([N-2], [N-3, N-2])] = [1, 3.5]
        else:
            A = 3.5
    for i in range(1, N-2):
        A[np.ix_([i], [i-1, i, i+1])] = [1, 4, 1]
    # Construct de Boor Control Points from A matrix
    d_pts = np.zeros((N+3, k))
    for col in range(0, k):
        x = np.zeros((max(N-1, 1), 1))
        if N > 2:
            # Compute start / end conditions
            if natural:
                x[N-2, 0] = 6*points_array[-2, col] - points_array[-1, col]
                x[0, 0] = 6*points_array[1, col] - points_array[0, col]
            else:
                x[N-2, 0] = 6*points_array[-2, col] - 1.5*dN[0, col]
                x[0, 0] = 6*points_array[1, col] - 1.5*d0[0, col]
            x[range(1, N-3+1), 0] = 6*points_array[range(2, N-2+1), col]
            # Solve bezier interpolation
            d_pts[2:N+1, col] = np.linalg.solve(A, x).T
        else:
            # Compute start / end conditions
            if natural:
                x[0, 0] = 6*points_array[1, col] - points_array[0, col]
            else:
                x[0, 0] = 6*points_array[1, col] - 1.5*d0[col]
            # Solve bezier interpolation
            d_pts[2, col] = x / A
    # Store off start and end positions
    d_pts[0, :] = points_array[0, :]
    d_pts[-1, :] = points_array[-1, :]
    # Compute the second to last de Boor point based on end conditions
    if natural:
        one_third = (1.0/3.0)
        two_thirds = (2.0/3.0)
        d_pts[1, :] = (two_thirds)*points_array[0, :] + (one_third)*d_pts[2, :]
        d_pts[N+1, :] = ((one_third)*d_pts[-3, :] +
                         (two_thirds)*points_array[-1, :])
    else:
        d_pts[1, :] = d0
        d_pts[N+1, :] = dN
    return d_pts


def bezier_coefficients(points_array, d_pts):
    """
    Compute the Bezier coefficients for a given
    set for user-supplied control pts and
    de Boor control pts.

    These B coeffs are used to compute the cubic
    splines for each cubic spline segment as
    follows (where t is a percentage of time between
    b_coeff segments):
    C(t) = (1 - t)^3*b0 + 3(1 - t)*b1
            + 3(1 - t)*t^2*b2 + t^3*b3

    params:
        points_array: array of user-supplied control points
            numpy.array of size N by k
            N is the number of control points
            k is the number of dimensions for each point

        d_pts: array of de Boor control points
            numpy.array of size N+3 by k

    returns:
        b_coeffs: k-dimensional array of 4 Bezier coefficients
            for every control point
            numpy.array of size N by 4 by k
    """
    (rows, k) = np.shape(points_array)
    N = rows - 1  # N minus 1 because points array includes x_0
    b_coeffs = np.zeros(shape=(k, N, 4))
    for i in range(0, N):
        points_array_i = i+1
        d_pts_i = i + 2
        if i == 0:
            for axis_pos in range(0, k):
                b_coeffs[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                        axis_pos]
                b_coeffs[axis_pos, i, 1] = d_pts[d_pts_i - 1, axis_pos]
                b_coeffs[axis_pos, i, 2] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                            + 0.5 * d_pts[d_pts_i, axis_pos])
                b_coeffs[axis_pos, i, 3] = points_array[points_array_i,
                                                        axis_pos]
        elif i == N-1:
            for axis_pos in range(0, k):
                b_coeffs[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                        axis_pos]
                b_coeffs[axis_pos, i, 1] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                            + 0.5 * d_pts[d_pts_i, axis_pos])
                b_coeffs[axis_pos, i, 2] = d_pts[d_pts_i, axis_pos]
                b_coeffs[axis_pos, i, 3] = points_array[points_array_i,
                                                        axis_pos]
        else:
            for axis_pos in range(0, k):
                b_coeffs[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                        axis_pos]
                b_coeffs[axis_pos, i, 1] = (2.0/3.0 * d_pts[d_pts_i - 1,
                                                            axis_pos]
                                            + 1.0/3.0 * d_pts[d_pts_i,
                                                              axis_pos])
                b_coeffs[axis_pos, i, 2] = (1.0/3.0 * d_pts[d_pts_i - 1,
                                                            axis_pos]
                                            + 2.0/3.0 * d_pts[d_pts_i,
                                                              axis_pos])
                b_coeffs[axis_pos, i, 3] = points_array[points_array_i,
                                                        axis_pos]

    return b_coeffs


def _cubic_spline_point(b_coeff, t):
    """
    Internal convenience function for calculating
    a k-dimensional point defined by the supplied
    Bezier coefficients. Finds the point that
    describes the current position along the bezier
    segment for k dimensions.

    params:
        b_coeff => b0...b3: Four k-dimensional Bezier
            coefficients each one is a numpy.array
            of size k by 1, so
            b_coeff is a numpy array of size k by 4
            k is the number of dimensions for each
            coefficient
        t: percentage of time elapsed for this segment
            0 <= int <= 1.0

    returns:
        current position in k dimensions
            numpy.array of size 1 by k
    """
    return (pow((1-t), 3)*b_coeff[:, 0] +
            3*pow((1-t), 2)*t*b_coeff[:, 1] +
            3*(1-t)*pow(t, 2)*b_coeff[:, 2] +
            pow(t, 3)*b_coeff[:, 3]
            )


def bezier_point(b_coeffs, b_index, t):
    """
    Finds the k values that describe the current
    position along the bezier curve for k dimensions.

    params:
        b_coeffs: k-dimensional array
            for every control point with 4 Bezier coefficients
            numpy.array of size k by N by 4
            N is the number of control points
            k is the number of dimensions for each point
        b_index: index position out between two of
            the N b_coeffs for this point in time
            int
        t: percentage of time that has passed between
            the two control points
            0 <= int <= 1.0

    returns:
        b_point: current position in k dimensions
            numpy.array of size 1 by k
    """
    if b_index <= 0:
        b_point = b_coeffs[:, 0, 0]
    elif b_index > b_coeffs.shape[1]:
        b_point = b_coeffs[:, -1, -1]
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        b_coeff_set = b_coeffs[:, b_index-1, range(4)]
        b_point = _cubic_spline_point(b_coeff_set, t)
    return b_point


def bezier_curve(b_coeffs, num_intervals):
    """
    Iterpolation of the entire Bezier curve at once,
    using a specified number of intervals between
    control points (encapsulated by b_coeffs).

    params:
        b_coeffs: k-dimensional array of 4 Bezier coefficients
            for every control point
            numpy.array of size N by 4 by k
            N is the number of control points
            k is the number of dimensions for each point
        num_intervals: the number of intervals between
            control points
            int > 0

    returns:
        b_curve: positions along the bezier curve in k-dimensions
            numpy.array of size N*num_interval+1  by k
            (the +1 is to include the start position on the curve)
    """
    assert num_intervals > 0,\
        "Invalid number of intervals chosen (must be greater than 0)"
    interval = 1.0 / num_intervals
    (num_axes, num_bpts, _) = np.shape(b_coeffs)
    b_curve = np.zeros((num_bpts*num_intervals+1, num_axes))
    # Copy out initial point
    b_curve[0, :] = b_coeffs[:, 0, 0]
    for current_bpt in range(num_bpts):
            b_coeff_set = b_coeffs[:, current_bpt, range(4)]
            for iteration, t in enumerate(np.linspace(interval, 1,
                                                      num_intervals)):
                b_curve[(current_bpt *
                         num_intervals +
                         iteration+1), :] = _cubic_spline_point(b_coeff_set, t)
    return b_curve
