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
from math import *


vit= 2

################################################################################
def ImuCallback(odom_message):
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.steering_angle)
	global Qx, Qy, Qz, Qw, Xp,Yp, Psip,Psi,X,Y
	X=odom_message.pose.pose.position.x	
	Y=odom_message.pose.pose.position.y	
	Qx=odom_message.pose.pose.orientation.x
	Qy=odom_message.pose.pose.orientation.y
	Qz=odom_message.pose.pose.orientation.z
	Qw=odom_message.pose.pose.orientation.w
	Xp=odom_message.twist.twist.linear.x
	Yp=odom_message.twist.twist.linear.y
	Psi=math.atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))

################################################################################

def talker():
	k1=1;
	k2=1;
	k3=3;
	k4=3;
	L=1.7;
	dt=0.001;
	cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=1/dt)
	rospy.Subscriber("/IMU", Odometry, ImuCallback)
	rospy.init_node('LQR', anonymous=True)
	r = rospy.Rate(1/dt) # 10hz
	vit=1
	x0=0
	y0=0
	theta0=0
	beta0=0*pi/180
	betat=0
	z1=X-x0;
	z2=Y-y0;
	print(Psi)
	z3=tan(Psi-theta0);
	z4=(tan(betat)-cos(Psi-theta0)*tan(beta0))/(L*cos(Psi-theta0)**3)+k2*z2;
	cmd=cmd_drive()
	cmd.linear_speed=vit
	cmd.steering_angle_rear=0
	t0=rospy.get_time()
	u1=vit
	u1t=u1
	u2=0
	u2t=u2
	#betat=atan((z4-k2*z2)*(L*cos(Psi-theta0)**3)+cos(Psi-theta0)*tan(beta))

	exp=(u1/L*tan(beta0))*z2+u1t*cos(atan(z3))-u1
	eyp=-(u1/L*tan(beta0))*z1+u1t*sin(atan(z3))
	ethetap=u1t/L*tan(betat)-u1/L*tan(beta0)
	w1=u1t*cos(atan(z3))-u1	
	w2=k2*eyp+(  3*tan(betat)/cos(atan(z3))-2*tan(beta0)    )*sin(atan(z3))/(  L*cos(atan(z3))**3  )*ethetap-u2/(  L*cos(beta0)**2*cos(atan(z3))**2  )+u2t/(  L*cos(betat)**2*cos(atan(z3))**3  )
	
	while not rospy.is_shutdown():

		#t=rospy.get_time()-t0

		dz1=(u1/L*tan(beta0))*z2+w1;
		dz2=-(u1/L*tan(beta0))*z1+u1*z3+w1*z3;
		dz3=-k2*u1*z2+u1*z4+w1*(z4-k2*z2+(1+z3**2)*tan(beta0)/L);	
		dz4=w2;
		z1=z1+dt*dz1;
		z2=z2+dt*dz2;
		z3=z3+dt*dz3;
		z4=z4+dt*dz4;
		w1=-k1*abs(u1)*(z1+z3/k2*(z4+(1+z3**2)*tan(beta0)/L));
		w2=-k3*u1*z3-k4*abs(u1)*z4;
		ex=z1
		ey=z2
		etheta=atan(z3)
		u1t=((w1+u1)/cos(etheta))	
		betat=(atan((z4-k2*ey)*(L*cos(etheta)**3)+cos(etheta)*tan(beta0)))
		cmd.steering_angle_front=betat
		cmd.linear_speed=u1t
		cmd_publisher.publish(cmd)
		r.sleep()
	rospy.loginfo("out of the loop")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
