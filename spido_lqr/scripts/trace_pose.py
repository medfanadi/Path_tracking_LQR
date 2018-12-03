#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from math import *
import numpy as np
import matplotlib.pyplot as plt
#from cardraw import draw_car
import shapely.geometry as geom
from scipy import interpolate
from scipy import signal
from scipy import fftpack

import shapely.geometry as geom
from scipy import spatial
#from ButterFilter import butter_lowpass_filter

######Trajectory plot   #################################################################
kk=180/pi


res=np.loadtxt('/home/summit/Spido_ws/src/spido_lqr/scripts/TraRef.txt')

res2=np.loadtxt('/home/summit/Spido_ws/src/spido_lqr/scripts/TrajLQR.txt')

#plt.plot(res2[:,1],res2[:,2], '+r', linewidth=3.0)


plt.plot(res[:,1],res[:,2], '+b', linewidth=3.0)
plt.axis('equal')
plt.xlabel('x(m)', fontsize=20)
plt.ylabel('y(m)', fontsize=20)

#########################################################################################

plt.grid(True)
plt.legend(["Reference path"])
plt.show()


#################################################################################
plt.plot(res[:,0],res[:,3]*180/pi, 'r', linewidth=3.0)
plt.show()
