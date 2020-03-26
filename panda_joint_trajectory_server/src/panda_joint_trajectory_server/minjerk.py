#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Kei Okada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

"""
This library implement Minimum Jerk trajectory generation, a.k.a Hoff
& Arbib, described in the documents
http://mplab.ucsd.edu/tutorials/minimumJerk.pdf (you can find copy of
this at http://www.shadmehrlab.org/book/minimumjerk.pdf)

Hoff B, Arbib MA (1992) A model of the effects of speed, accuracy, and
perturbation on visually guided reaching. In: Control of arm movement
in space: neurophysiological and computational approaches
(Caminiti R, Johnson PB, Burnod Y, eds), pp 285-306.

~~~~~~~~~~~~~~~~~~~~~~~~ Min Jerk ~~~~~~~~~~~~~~~~~~~~~~~~
A library for computing minimum jerk trajectory for an arbitrary
set of control points in R2, R3, up to RN space.

  x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 x'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
x''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3

Solve this problem from boundary conditions of x(0), x'(0), x''(0) and
x(T), x'(T), x''(T).

ex. usage:

import numpy
import minjerk
points_array = numpy.array([[1, 2, 3], [4, 4, 4],
                            [6, 4, 6], [2, 5, 6],
                            [5, 6, 7]])
m_coeffs = minjerk.minjerk_coefficients(points_array)
m_curve = minjerk.minjerk_trajectory(m_coeffs, 10)
#  plotting example
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')
#plot bezier curve
ax.plot(m_curve[:,0], m_curve[:,1], m_curve[:,2], 'r')
#plot specified points
ax.plot(points_array[:,0], points_array[:,1], points_array[:,2], 'g*')
ax.set_title("Minimum Jerk Trajectory")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend(["Minimum Jerk", "Control Points"], loc=2)
plt.show()
"""

import numpy as np


def minjerk_coefficients(points_array, duration_array=None):
     """
     Compute the min-jerk coefficients for a given set for user-supplied control pts
     
     params:
        points_array: array of user-supplied control points
            numpy.array of size N by k
            N is the number of control points
            k is the number of dimensions for each point
        duration_array: array of user-supplied control duration of ech segment
            numpy.array of size N-1
            N is the number of control points

     returns:
       m_coeffs:  k-dimensional array of N-1 x (6 coefficients + 1 duration of each segment)
            numpy.array of size N-1 by (6+1) by k
     """
     (rows, k) = np.shape(points_array)
     N = rows - 1  # N minus 1 because points array includes x_0
     m_coeffs = np.zeros(shape=(k, N, 7))
     x = points_array[0]
     v = np.zeros(k)
     a = np.zeros(k)
     if duration_array == None:
          duration_array = np.array([1.0]*N)
     assert len(duration_array) == N,\
          "Invalid number of intervals chosen (must be equal to N+1={})".format(N)
     for i in range(0, N):
          gx = points_array[i+1];
          t = duration_array[i]
          if i == N-1:
               gv = np.zeros(k)
          else:
               t0 = t
               t1 = duration_array[i+1]
               d0 = points_array[i+1] - points_array[i]
               d1 = points_array[i+2] - points_array[i+1]
               v0 = d0 / t0
               v1 = d1 / t1
               gv = np.where(np.multiply(v0, v1)>=1e-10, 0.5 * ( v0 + v1 ), np.zeros(k)) # 0 + eps
          ga = np.zeros(k)

          A=(gx-(x+v*t+(a/2.0)*t*t))/(t*t*t);
          B=(gv-(v+a*t))/(t*t);
          C=(ga-a)/t;

          a0=x;
          a1=v;
          a2=a/2.0;
          a3=10*A-4*B+0.5*C;
          a4=(-15*A+7*B-C)/t;
          a5=(6*A-3*B+0.5*C)/(t*t);

          x = gx
          v = gv

          m_coeffs[:,i,0] = a0
          m_coeffs[:,i,1] = a1
          m_coeffs[:,i,2] = a2
          m_coeffs[:,i,3] = a3
          m_coeffs[:,i,4] = a4
          m_coeffs[:,i,5] = a5
          m_coeffs[:,i,6] = t
     return m_coeffs

def minjerk_trajectory(m_coeffs, num_intervals, duration_array=None):
    """
    Iterpolation of the entire minimum jerk trajectory at once,
    using a specified number of intervals between
    control points (encapsulated by m_coeffs).

    params:
        m_coeffs: N-dimensional array of (6+1) x k  coefficients
            for every control point
            numpy.array of size N by (6 + 1) by k
            N is the number of control points
            k is the number of dimensions for each point
        num_intervals: the number of intervals between
            control points
            int > 0
        duration_array: array of user-supplied control duration of segment
            numpy.array of size N-1
            N is the number of control points

    returns:
        m_curve: positions along the minimum trajectory  in k-dimensions
            numpy.array of size N*num_interval+1  by k
            (the +1 is to include the start position on the curve)
    """
    assert num_intervals > 0,\
        "Invalid number of intervals chosen (must be greater than 0)"
    interval = 1.0 / num_intervals
    (num_axes, num_mpts, _) = np.shape(m_coeffs)
    m_curve = np.zeros((num_mpts*num_intervals+1, num_axes))
    # Copy out initial point
    m_curve[0, :] = m_coeffs[:, 0, 0]
    if duration_array == None:
         duration_array = np.array([1.0]*num_mpts)
    assert len(duration_array) == num_mpts,\
         "Invalid number of intervals chosen (must be equal to N={})".format(num_mpts)
    for current_mpt in range(num_mpts):
         m_coeff_set = m_coeffs[:, current_mpt, range(7)]
         for iteration, t in enumerate(np.linspace(interval, 1,
                                                   num_intervals)):
              m_curve[(current_mpt *
                       num_intervals +
                       iteration+1), :] = _minjerk_trajectory_point(m_coeff_set, t * duration_array[current_mpt])
    return m_curve
    
def _minjerk_trajectory_point(m_coeff, t):
    """
    Internal convenience function for calculating
    a k-dimensional point defined by the supplied
    minimum jerk coefficients. Finds the point that
    describes the current position along the minimum
    trajectory segment for k dimensions.

    params:
        m_coeff => m0...m3: Four k-dimensional minimum jerk
            coefficients each one is a numpy.array
            of size k by 1, so
            m_coeff is a numpy array of size k by (6+1)
            k is the number of dimensions for each
            coefficient
        t: percentage of time elapsed for this segment
            0 <= int <= 1.0

    returns:
        current position in k dimensions
            numpy.array of size 1 by k
    """
    a0 = m_coeff[:,0]
    a1 = m_coeff[:,1]
    a2 = m_coeff[:,2]
    a3 = m_coeff[:,3]
    a4 = m_coeff[:,4]
    a5 = m_coeff[:,5]
    tm = m_coeff[:,6]

    t = t * tm # input t is percentage of time elapsed for this segment, tm is the duration of this segment and to calculate x, v, a , t is the time[s] elapsed for this segment

    # calculate x, v, z at the time percentage  t
    # x=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
    x=a0+a1*t+a2*np.power(t,2)+a3*np.power(t,3)+a4*np.power(t,4)+a5*np.power(t,5);
    # v=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
    v=a1+2*a2*t+3*a3*np.power(t,2)+4*a4*np.power(t,3)+5*a5*np.power(t,4);
    # a=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;
    a=2*a2+6*a3*t+12*a4*np.power(t,2)+20*a5*np.power(t,3);

    return x

def minjerk_point(m_coeffs, m_index, t):
    """
    Finds the k values that describe the current
    position along the minjerk trajectory for k dimensions.

    params:
        m_coeffs: k-dimensional array
            for every control point with 6 Minimum Jerk coefficients and a segument duration
            numpy.array of size k by N by 7
            N is the number of control points
            k is the number of dimensions for each point
        m_index: index position out between two of
            the N b_coeffs for this point in time
            int
        t: percentage of time that has passed between
            the two control points
            0 <= int <= 1.0

    returns:
        m_point: current position in k dimensions
            numpy.array of size 1 by k
    """
    if m_index <= 0:
        m_point = m_coeffs[:, 0, 0]
    elif m_index > m_coeffs.shape[1]:
        t = 1
        m_coeff_set = m_coeffs[:,m_coeffs.shape[1]-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        m_coeff_set = m_coeffs[:,m_index-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    return m_point
