import rospy
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_car
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point
import tf

from matplotlib import pyplot
import numpy as np
from math import fabs, tan, cos, sin, hypot, pi, atan2, atan, copysign
import matplotlib.pyplot as plt

import sys

#from filtrage import filtrage

def curvature():

    Traj=np.loadtxt('/home/summit/Spido_ws/src/spido_lqr/scripts/TraRef.txt')

    tarjT = Traj[:,0]
    trajX = Traj[:,1]
    trajY = Traj[:,2]
    Yaw = Traj[:,3]
    Psip= Traj[:,3]
    cpt = trajX.size

    s_vect = np.zeros(cpt)
    theta_s = np.zeros(cpt)
    c = np.zeros(cpt)

# s
    for i in range(1, cpt):
        s_vect[i] = s_vect[i-1] + hypot(trajX[i]-trajX[i-1],trajY[i]-trajY[i-1])


# theta_s
    for i in range(cpt):
        if i<cpt-1:
            angle = atan2(trajY[i+1]-trajY[i],trajX[i+1]-trajX[i])
            if angle < 0 :
                angle += 2*pi
                theta_s[i] = angle
            else:
                angle = theta_s[cpt-2]
                if angle < 0 :
                    angle += 2*pi
                    theta_s[i] = angle

# c
    for i in range(cpt):
        if i<cpt-1:
                c[i] = (theta_s[i+1]-theta_s[i])/(s_vect[i+1]-s_vect[i])
        elif i == cpt-1:
                c[i] = c[cpt-2]


    #TrajRe=np.array([[tarjT],[trajX],[trajY],[Yaw],[s_vect,c]])
    return tarjT,trajX,trajY,Yaw,Psip,s_vect,c


path=curvature()
print path
#plt.plot(path[0],path[5], 'r', linewidth=3.0)
#plt.show()
