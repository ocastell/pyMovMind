# -*- coding: utf-8 -*-

"""Copyright 2014 Roger R Labbe Jr.

filterpy library.
http://github.com/rlabbe/filterpy

This is licensed under an MIT license. See the readme.MD file
for more information.
"""


from __future__ import (absolute_import, division, print_function,
                        unicode_literals)


import numpy.random as random
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
from filterpy.leastsq import LeastSquaresFilter

from filterpy.gh import GHFilter



def near_equal(x,y, e=1.e-14):
    return abs(x-y) < e


class LeastSquaresFilterOriginal(object):
    """Implements a Least Squares recursive filter. Formulation is per
    Zarchan [1].

    Filter may be of order 0 to 2. Order 0 assumes the value being tracked is
    a constant, order 1 assumes that it moves in a line, and order 2 assumes
    that it is tracking a second order polynomial.

    It is implemented to be directly callable like a function. See examples.

    Examples
    --------

    lsq = LeastSquaresFilter(dt=0.1, order=1, noise_variance=2.3)

    while True:
        z = sensor_reading()  # get a measurement
        x = lsq(z)            # get the filtered estimate.
        print('error: {}, velocity error: {}'.format(lsq.error, lsq.derror))


    Member Variables
    ----------------

    n : int
        step in the recursion. 0 prior to first call, 1 after the first call,
        etc.

    K1,K2,K3 : float
        Gains for the filter. K1 for all orders, K2 for orders 0 and 1, and
        K3 for order 2

    x, dx, ddx: type(z)
        estimate(s) of the output. 'd' denotes derivative, so 'dx' is the first
        derivative of x, 'ddx' is the second derivative.


    References
    ----------
    [1] Zarchan and Musoff. "Fundamentals of Kalman Filtering: A Practical
        Approach." Third Edition. AIAA, 2009.

    """


    def __init__(self, dt, order, noise_variance=0.):
        """ Least Squares filter of order 0 to 2.

        Parameters
        ----------
        dt : float
           time step per update

        order : int
            order of filter 0..2

        noise_variance : float
            variance in x. This allows us to calculate the error of the filter,
            it does not influence the filter output.
        """

        assert order >= 0
        assert order <= 2

        self.reset()

        self.dt = dt
        self.dt2 = dt**2

        self.sigma = noise_variance
        self._order = order


    def reset(self):
        """ reset filter back to state at time of construction"""

        self.n = 0 #nth step in the recursion
        self.x = 0.
        self.error = 0.
        self.derror = 0.
        self.dderror = 0.
        self.dx = 0.
        self.ddx = 0.
        self.K1 = 0
        self.K2 = 0
        self.K3 = 0


    def __call__(self, z):
        self.n += 1
        n = self.n
        dt = self.dt
        dt2 = self.dt2

        if self._order == 0:
            self.K1 = 1./n
            residual =  z - self.x
            self.x = self.x + residual * self.K1
            self.error = self.sigma/sqrt(n)

        elif self._order == 1:
            self.K1 = 2*(2*n-1) / (n*(n+1))
            self.K2 = 6 / (n*(n+1)*dt)

            residual =  z - self.x - self.dx*dt
            self.x = self.x + self.dx*dt + self.K1*residual
            self.dx = self.dx + self.K2*residual

            if n > 1:
                self.error = self.sigma*sqrt(2.*(2*n-1)/(n*(n+1)))
                self.derror = self.sigma*sqrt(12./(n*(n*n-1)*dt*dt))

        else:
            den = n*(n+1)*(n+2)
            self.K1 = 3*(3*n**2 - 3*n + 2) / den
            self.K2 = 18*(2*n-1) / (den*dt)
            self.K3 = 60./ (den*dt2)

            residual =  z - self.x - self.dx*dt - .5*self.ddx*dt2
            self.x   += self.dx*dt  + .5*self.ddx*dt2 +self. K1 * residual
            self.dx  += self.ddx*dt + self.K2*residual
            self.ddx += self.K3*residual

            if n >= 3:
                self.error = self.sigma*sqrt(3*(3*n*n-3*n+2)/(n*(n+1)*(n+2)))
                self.derror = self.sigma*sqrt(12*(16*n*n-30*n+11) /
                                              (n*(n*n-1)*(n*n-4)*dt2))
                self.dderror = self.sigma*sqrt(720/(n*(n*n-1)*(n*n-4)*dt2*dt2))

        return self.x


    def standard_deviation(self):
        if self.n == 0:
            return 0.

        if self._order == 0:
            return 1./sqrt(self)

        elif self._order == 1:
            pass


    def __repr__(self):
        return 'LeastSquareFilter x={}, dx={}, ddx={}'.format(
               self.x, self.dx, self.ddx)


def test_lsq():
    """ implements alternative version of first order Least Squares filter
    using g-h filter formulation and uses it to check the output of the
    LeastSquaresFilter class."""

    gh = GHFilter(x=0, dx=0, dt=1, g=.5, h=0.02)
    lsq = LeastSquaresFilterOriginal(dt=1, order=1)
    lsq2 = LeastSquaresFilter(dt=1, order=1)
    zs = [x+random.randn() for x in range(0,100)]

    xs = []
    lsq_xs= []
    for i,z in enumerate(zs):
        g = 2*(2*i + 1) / ((i+2)*(i+1))
        h = 6 / ((i+2)*(i+1))


        x,dx = gh.update(z,g,h)
        lx = lsq(z)
        lsq_xs.append(lx)

        x2 = lsq2.update(z)
        assert near_equal(x2[0], lx, 1.e-13)
        xs.append(x)


    plt.plot(xs)
    plt.plot(lsq_xs)

    for x,y in zip(xs, lsq_xs):
        r = x-y
        assert r < 1.e-8


def test_first_order ():
    ''' data and example from Zarchan, page 105-6'''

    lsf = LeastSquaresFilter(dt=1, order=1)

    xs = [1.2, .2, 2.9, 2.1]
    ys = []
    for x in xs:
        ys.append (lsf.update(x)[0])

    plt.plot(xs,c='b')
    plt.plot(ys, c='g')
    plt.plot([0,len(xs)-1], [ys[0], ys[-1]])



def test_second_order ():
    ''' data and example from Zarchan, page 114'''

    lsf = LeastSquaresFilter(1,order=2)
    lsf0 = LeastSquaresFilterOriginal(1,order=2)

    xs = [1.2, .2, 2.9, 2.1]
    ys = []
    for x in xs:
        y = lsf.update(x)[0]
        y0 = lsf0(x)
        assert near_equal(y, y0)
        ys.append (y)


    plt.scatter(range(len(xs)), xs,c='r', marker='+')
    plt.plot(ys, c='g')
    plt.plot([0,len(xs)-1], [ys[0], ys[-1]], c='b')


def test_fig_3_8():
    """ figure 3.8 in Zarchan, p. 108"""
    lsf = LeastSquaresFilter(0.1, order=1)
    lsf0 = LeastSquaresFilterOriginal(0.1, order=1)

    xs = [x+3 + random.randn() for x in np.arange (0,10, 0.1)]
    ys = []
    for x in xs:
        y0 = lsf0(x)
        y = lsf.update(x)[0]
        assert near_equal(y, y0)
        ys.append (y)

    plt.plot(xs)
    plt.plot(ys)


def test_listing_3_4():
    """ listing 3.4 in Zarchan, p. 117"""

    lsf = LeastSquaresFilter(0.1, order=2)

    xs = [5*x*x -x + 2 + 30*random.randn() for x in np.arange (0,10, 0.1)]
    ys = []
    for x in xs:
        ys.append (lsf.update(x)[0])

    plt.plot(xs)
    plt.plot(ys)


if __name__ == "__main__":
    #test_listing_3_4()

    #test_second_order()
    #fig_3_8()

    test_second_order()