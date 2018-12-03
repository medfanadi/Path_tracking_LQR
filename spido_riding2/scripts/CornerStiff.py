#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import numpy as np
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
#from scipy import *
from scipy.linalg import solve_continuous_are
import time
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_drive
from std_msgs.msg import Float64, String, Int32
from sensor_msgs.msg import Imu, NavSatFix
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round



from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial


import subprocess
import rospy
import numpy.linalg as alg
#import matpbandlotlib.pyplot as plt
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


# == Paramètres du modèle ======================================================
kkk=pi/180
Rt=6371000;
# latitude_init=48.80564077; //sur le terrain réel
# longitude_init=2.07643527;
latitude_init=39.5080331#39.5080322117 #sur gazebo
longitude_init=-0.4619816#-0.46198057533;
vit=5
#Cf      = 15000   #rigidité de dérive du train avant
#Cr      = 15000    # rigidité de dérive du train arrière
masse       = 880
moment      = 86.7
a       = 0.85
b       = 0.85
d       = 0.5
######
#k=1/20  # à calculer à chaque pas de calcul ???? .txt
#######"
Xp=0
Yp=0
Psip=0
Psi=0
X=0
Y=0


################################################################################
def get_model_Ac_matrix() :

    ac11=0
    ac12=-vit
    ac21=0
    ac22=0


    return np.array([   [a11,a12],
                        [a21,a22]])

################################################################################
def get_model_Bc_matrix(Vy,Vpsi,deltaf,deltar) :

   b11=-2*(Vy+a*Vpsi-vit*deltaf)/(masse*vit)
   b12=-2*(Vy+b*Vpsi-vit*deltar)/(masse*vit)
   b21=-2*(a*Vy+a*a*Vpsi-a*vit*deltaf)/(moment*vit)
   b22=-2*(b*Vy+b*b*Vpsi-b*vit*deltar)/(moment*vit)


   return np.array([[b11,b12],
                        [b21,b22]])
################################################################################

################################################################################
def get_model_A_matrix(k,Cf,Cr) :

    a11=-2*(Cf+Cr)/(masse*vit)
    a12=-2*(a*Cf-b*Cr)/(masse*vit)-vit
    a13=0
    a14=0
    a21=-2*(a*Cf-b*Cr)/(vit*moment)
    a22=-2*(a*a*Cf+b*b*Cr)/(vit*moment)
    a23=0
    a24=0
    a31=1
    a32=0
    a33=0
    a34=vit
    a41=0
    a42=1
    a43=k*k*vit
    a44=0


    return np.array([   [a11,a12, a13, a14],
                        [a21,a22, a23, a24],
                        [a31,a32, a33, a34],
                        [a41,a42, a43, a44]])

################################################################################
def get_model_B_matrix(Cf,Cr) :

   b11=2*Cf/masse
   b12=2*Cr/masse
   b21=2*a*Cf/moment
   b22=-2*b*Cr/moment
   b31=0
   b32=0
   b41=0
   b42=0

   return np.array([[b11,b12],
                        [b21,b22],
                        [b31,b32],
                        [b41,b42]])
################################################################################

C=np.array([[10], [1000]])
L=np.array([[10, 0],
                    [0, 10]])
Cf=C[0]
Cr=C[1]
get_model_B_matrix(Cf,Cr)
