#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import numpy as np

from std_msgs.msg import Float64, String, Int32
#from scipy.signal import place_poles
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round



from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial


import subprocess
import rospy
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
from scipy import *

from scipy.linalg import solve_continuous_are
import time

from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc

from matplotlib.collections import PatchCollection
import shapely.geometry as geom
from scipy import spatial


from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


######
#k=1/20  # à calculer à chaque pas de calcul ???? .txt
#######"
ey=0
epsi=0
Psip=0
Psi=0
X=0
Y=0
vy=0
bf=0
br=0
Xref=0
Yref=0
Psiref=0
vpsiObs=0
eyObs=0
epsiObs=0




################################################################################
def Callback(message):
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.steering_angle)
	global X,Y,Psi,ey,epsi,vy,Psip,bf,br,Xref,Yref,Psiref
	X=message.data[0]
	Y=message.data[1]
	Psi=message.data[2]
	ey=message.data[3]
	epsi=message.data[4]
	vy=message.data[5]
	Psip=message.data[6]
	bf=message.data[7]
	br=message.data[8]
	Xref=message.data[9]
	Yref=message.data[10]
	Psiref=message.data[11]
    #vpsiObs=message.data[12]
	#eyObs=message.data[13]
	#epsiObs=message.data[14]
################################################################################

################################################################################

################################################################################
def recorder():
	file = open("/home/summit/Spido_ws/recorded_files/steerLQR.txt","w")
	rospy.Subscriber('floats', numpy_msg(Floats), Callback)

	rospy.init_node('record', anonymous=True)
	r = rospy.Rate(200) # 10hz
	simul_time = rospy.get_param('~simulation_time', '10')

	while(abs(X)<0.001):
		i=0
	t0=rospy.get_time()
	while (rospy.get_time()-t0<=simul_time):
		#print (rospy.get_time()-t0,X,Y,ey,epsi,'.....')
		file.write(' '.join((str(rospy.get_time()-t0), str(X),str(Y), str(Psi),str(ey),str(epsi), str(vy),str(Psip),str(bf), str(br),str(Xref),str(Yref),str(Psiref))) +'\n')
		r.sleep()
	file.close()
if __name__ == '__main__':
    try:
        recorder()
    except rospy.ROSInterruptException: pass
