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


kkk=pi/180
Rt=6371000;
# latitude_init=48.80564077; //sur le terrain réel
# longitude_init=2.07643527;
latitude_init=39.5080331#39.5080322117 #sur gazebo
longitude_init=-0.4619816#-0.46198057533;
vit= 5
Cf      = 15000   #rigidité de dérive du train avant
Cr      = 15000    # rigidité de dérive du train arrière
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
	#Psi=Psi+80*0.017453293

################################################################################
def retour_gps (msg):
	global longitude,latitude, X ,Y
	longitude= msg.longitude;
	latitude= msg.latitude;
	X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
	Y=Rt*(latitude - latitude_init)*kkk

################################################################################
def get_model_A_matrix(k) :

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
	data_pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
	rospy.Subscriber("/IMU", Odometry, ImuCallback)
	rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
	rospy.init_node('LQR', anonymous=True)
	r = rospy.Rate(200) # 10hz

	simul_time =rospy.get_param('~simulation_time', '10')
# == Dynamic Model ======================================================





# == Steady State   ======================================================

	B=get_model_B_matrix()


# == LQR control (Gains)   ======================================================


	Q=np.array([   [0.0001, 0,0, 0],
                        [0,0.0001, 0, 0],
                        [0,0, 0.0001, 0],
                        [0,0,0, 0.0001]])

	R=np.array([   [20000,0],
                        [0,20000]])



	C=np.array([[1, 0, 0 ,0],
    [0, 1, 0 ,0]])




	#print(M)



# == Observer  ======================================================


	#print(xhat)
	Gammax=0.01*eye(4,4)

	dt=0.01  #periode d'échantionnage

	Gammaalpha=dt*0.00001*eye(4,4)
	Gammabeta=dt**2*eye(2,2)*0.00001


	while(abs(X)<0.001):
		i=0
	mat=np.loadtxt('/home/summit/Spido_ws/recorded_files/mat.txt')
	#figure(1)
	#plt.plot(mat[:,1:2],mat[:,2:3])
	mat[:,1]=mat[:,1]-mat[0,1]
	mat[:,2]=mat[:,2]-mat[0,2]
	dpsi=Psi-mat[0,3]
	#figure(2)
	#plt.plot(mat[:,0],mat[:,3])
	mat[:,3]=mat[:,3]+dpsi
	RR=np.array([[cos(dpsi),-sin(dpsi),X],[sin(dpsi) ,cos(dpsi) ,Y],[0 ,0 ,1]])
	FF=np.array([np.transpose(mat[:,1]),np.transpose(mat[:,2]),np.ones((1,len(mat)))])
	A=RR.dot(FF)
	mat[:,1:2]=np.transpose(A[0])
	mat[:,2:3]=np.transpose(A[1])
	#figure(1)
	#plt.plot(mat[:,1:2],mat[:,2:3])
	#figure(2)
	#plt.plot(mat[:,0],mat[:,3])
	#plt.show()



	Psiref=mat[0,3]
	Psipref=0#mat[0,6]
	xref=mat[0,1]
	yref=mat[0,2]
	ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
	epsi=(Psi-Psiref)
	xhat=np.array([[0],[Psip],[ey],[epsi]])

	#mat[:,3]=mat[:,3]-mat[0,3]+Psi
	#coords=mat[:,1:3]
	line = geom.LineString(mat[:,1:3])
	t0=  rospy.get_time()
	#pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)



	#file = open("steerLQR.txt","w")


	while (rospy.get_time()-t0<=simul_time):
		#plt.plot(X,Y,xref,yref)
		cmd=cmd_drive()
		#subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)
		point = geom.Point(X,Y)
		nearest_pt=line.interpolate(line.project(point))
		distance,index = spatial.KDTree(mat[:,1:3]).query(nearest_pt)
		xref=mat[index,1]
		yref=mat[index,2]
		Psiref=mat[index,3]
		k=mat[index,7]
		vyref=0#mat[index,5]
		Psipref=0#mat[index,6]
		ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
		epsi=(Psi-Psiref)
		#vy=-Xp*sin(Psi)+Yp*cos(Psi)
		vy=xhat[0,0]
        #vpsiObs=xhat[1,0]
        #eyObs=xhat[2,0]
        #epsiObs=xhat[3,0]
        x=np.array([[vy],[Psip],[ey],[epsi]])
        yd=np.array([[vyref],[Psipref]])

        y=C.dot(x)
	        #print('u',u)
        A=get_model_A_matrix(k)
        Vyss=-(k*vit*(A[(0,1)]*B[(1,0)] - A[(1,1)]*B[(0,0)] - A[(0,1)]*B[(1,1)] + A[(1,1)]*B[(0,1)]))/(A[(0,0)]*B[(1,0)] - A[(1,0)]*B[(0,0)] - A[(0,0)]*B[(1,1)] +  A[(1,0)]*B[(0,1)])
        Vpsiss=vit*k
        eyss=0;
        epsiss=-Vyss/vit;

        bfss=-(k*vit*(A[(0,0)]*A[(1,1)] - A[(0,1)]*A[(1,0)]))/(A[(0,0)]*B[(1,0)] - A[(1,0)]*B[(0,0)] - A[(0,0)]*B[(1,1)] +  A[(1,0)]*B[(0,1)])
        brss=-bfss;

        xsss=np.array([[Vyss],[Vpsiss],[eyss],[epsiss]])
        usss=np.array([[bfss],[brss]])
        P=solve_continuous_are(A, B, Q, R)
        print(P)


        G=np.linalg.inv(R).dot(np.transpose(B)).dot(P)
	#print(G)
	#np.array([[-0.5,4.3e+13],[-0.5,-4.2e+13]])#
        M = pinv(C.dot(pinv(A-B.dot(G))).dot(B))

        u=M.dot(yd)-G.dot(x-xsss)+usss
		#u=M.dot(yd)-G.dot(x)
        print(u)
        xhat,Gammax=kalman(xhat,Gammax,dt*B.dot(u),y,Gammaalpha,Gammabeta,eye(4,4)+dt*A,C)
		#print('xhat',xhat)

		#vy1=-Xp*sin(Psi)+Yp*cos(Psi);# // vy:vitesse latérale expriméé dans le repère véhicule

		#;//-xpref*sin(psiref)+ypref*cos(psiref); // vy:vitesse latérale expriméé dans le repère véhicule
		#subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)
		#unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        cmd.linear_speed=vit
        cmd.steering_angle_front=u[0,0]
        cmd.steering_angle_rear=u[1,0]
        cmd_publisher.publish(cmd)


        posture = np.array([X,Y,Psi,ey,epsi,vy,Psip,u[0,0],u[1,0],xref,yref,Psiref], dtype=np.float32)
        data_pub.publish(posture)
		#file.write(' '.join((str(rospy.get_time()-t0), str(u[0,0]), str(u[1,0]))))
        r.sleep()
	cmd.steering_angle_front=0
	cmd.steering_angle_rear=0
	aa=vit
	while (aa>0):
		cmd=cmd_drive()
		aa=aa-0.1
		if aa<0:
			aa=0
		cmd.linear_speed=aa
		cmd_publisher.publish(cmd)

	#rospy.loginfo("out of the loop")
	#file.close()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
