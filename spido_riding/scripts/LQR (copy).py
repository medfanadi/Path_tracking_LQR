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
kkk=pi/180
Rt=6371000;
# latitude_init=48.80551814; //sur le terrain réel
# longitude_init=2.07650929;
latitude_init=39.5080322117; #sur gazebo
longitude_init=-0.46198057533;
vit= 2
Cf      = 15000   #rigidité de dérive du train avant
Cr      = 15000    # rigidité de dérive du train arrière
masse       = 880
moment      = 86.7
a       = 0.85
b       = 0.85
d       = 0.5
######
k=1/20  # à calculer à chaque pas de calcul ???? .txt
#######"
Xp=0
Yp=0
Psip=0
Psi=0
X=0
Y=0
#################################################################################
# Extended Kalman Filter Functions
#################################################################################

def kalman(x0,Gamma0,u,y,Gammaalpha,Gammabeta,A,C):
    S = C.dot(Gamma0) .dot(np.transpose(C)) + Gammabeta        
    Kal = Gamma0 .dot(np.transpose(C)) .dot(inv(S) )          
    ytilde = y - C .dot(x0 )       
    Gup = (eye(len(x0))-Kal .dot(C) ).dot(Gamma0)
    xup = x0 + Kal.dot(ytilde)
    Gamma1 = A .dot(Gup) .dot(np.transpose(A)) + Gammaalpha
    x1 = A .dot(xup) + u    
    return(x1,Gamma1)    
 
         
################################################################################
def ImuCallback(odom_message):
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.steering_angle)
	global Qx, Qy, Qz, Qw, Xp,Yp, Psip,Psi
	Qx=odom_message.pose.pose.orientation.x
	Qy=odom_message.pose.pose.orientation.y
	Qz=odom_message.pose.pose.orientation.z
	Qw=odom_message.pose.pose.orientation.w
	Xp=odom_message.twist.twist.linear.x
	Yp=odom_message.twist.twist.linear.y
   	Psip=odom_message.twist.twist.angular.z
	Psi=math.atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))

################################################################################
def retour_gps (msg):
	global longitude,latitude, X ,Y
	longitude= msg.longitude;
	latitude= msg.latitude;
	X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
	Y=Rt*(latitude - latitude_init)*kkk	
	
################################################################################
def get_model_A_matrix() :
 
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
def get_model_B_matrix() :
   
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
def talker():
	cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=10)
	rospy.Subscriber("/IMU", Odometry, ImuCallback)
	rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
	rospy.init_node('LQR', anonymous=True)
	r = rospy.Rate(10) # 10hz
	simul_time = rospy.get_param('~simulation_time', '10')
# == Dynamic Model ======================================================




# == Steady State   ======================================================
	A=get_model_A_matrix()
	B=get_model_B_matrix()

	Vyss=-(k*vit*(A[(0,1)]*B[(1,0)] - A[(1,1)]*B[(0,0)] - A[(0,1)]*B[(1,1)] + A[(1,1)]*B[(0,1)]))/(A[(0,0)]*B[(1,0)] - A[(1,0)]*B[(0,0)] - A[(0,0)]*B[(1,1)] +  A[(1,0)]*B[(0,1)])
	Vpsiss=vit*k
	eyss=0;
	epsiss=-Vyss/vit;

	bfss=-(k*vit*(A[(0,0)]*A[(1,1)] - A[(0,1)]*A[(1,0)]))/(A[(0,0)]*B[(1,0)] - A[(1,0)]*B[(0,0)] - A[(0,0)]*B[(1,1)] +  A[(1,0)]*B[(0,1)])
	brss=-bfss;

	xsss=np.array([[Vyss],[Vpsiss],[eyss],[epsiss]])
	usss=np.array([[bfss],[brss]])

# == LQR control (Gains)   ======================================================


	Q=np.array([   [0.0001, 0,0, 0],
                        [0,0.0001, 0, 0],
                        [0,0, 0.0001, 0],
                        [0,0,0, 0.0001]])

	R=np.array([   [20000,0],
                        [0,20000]])



	C=np.array([[1, 0, 0 ,0],
    [0, 1, 0 ,0]])




	P=solve_continuous_are(A, B, Q, R)
	#print(P)


	G=np.linalg.inv(R).dot(np.transpose(B)).dot(P)
	#print(G)
	#np.array([[-0.5,4.3e+13],[-0.5,-4.2e+13]])#
	M = pinv(C.dot(pinv(A-B.dot(G))).dot(B))
	#print(M)

	

# == Observer  ======================================================

	xhat=np.array([[0],[0],[1.5],[0]])
	#print(xhat)
	Gammax=0.01*eye(4,4)
	
	dt=0.01  #periode d'échantionnage

	Gammaalpha=dt*0.00001*eye(4,4)
	Gammabeta=dt**2*eye(2,2)*0.00001
	Psiref=0*pi/180
	Psipref=0*pi/180
	vyref=0
	mat=np.loadtxt('/home/bachir/spido_ws/src/spido_riding/scripts/mat.txt')
	coords=curve[:,1:3]
	line = geom.LineString(coords)
	t0=  rospy.get_time()

	while (  rospy.get_time()-t0<=simul_time):
		cmd=cmd_drive()
		point = geom.Point(X,Y)
		nearest_pt=line.interpolate(line.project(point))
		distance,index = spatial.KDTree(coords).query(nearest_pt)
		xref=coords[index,0]
		yref=coords[index,1]
		Psiref=curve[index,3]
		ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
		epsi=(Psi-Psiref)
		#vy=-Xp*sin(Psi)+Yp*cos(Psi)
		vy=xhat[0,0]
		x=np.array([[vy],[Psip],[ey],[epsi]])
		yd=np.array([[vyref],[Psipref]])
		
		y=C.dot(x)		
	        #print('u',u)
		u=M.dot(yd)-G.dot(x)
		xhat,Gammax=kalman(xhat,Gammax,dt*B.dot(u),y,Gammaalpha,Gammabeta,eye(4,4)+dt*A,C)
		#print('xhat',xhat)
		print(Psi*180/pi)
		vy1=-Xp*sin(Psi)+Yp*cos(Psi);# // vy:vitesse latérale expriméé dans le repère véhicule
		print(vy,vy1)
		#;//-xpref*sin(psiref)+ypref*cos(psiref); // vy:vitesse latérale expriméé dans le repère véhicule
		cmd.linear_speed=vit
		cmd.steering_angle_front=u[0,0]
		cmd.steering_angle_rear=u[1,0]
		cmd_publisher.publish(cmd)
		r.sleep()
	#rospy.loginfo("out of the loop")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
