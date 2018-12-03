#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from math import *
import numpy as np
import matplotlib.pyplot as plt

k1=1;
k2=1;
k3=3;
k4=3;
L=1.2;
dt=0.001;
T=np.arange(0, 30,dt)#0:dt:30;
u1=[]
u2=[]
for i in range(0, len(T)):
	if T[i]<=10:
		u1.append(1)
		u2.append(0)
	else:
		if T[i]<=20:
			u1.append(-1)
			u2.append(0.5*cos(2*pi*(T[i]-10)/5))
		else:
			u1.append(1)
			u2.append(0)
		
x=[]
y=[]
theta=[]
beta=[]
x.append(0)
y.append(0)
theta.append(0*pi/180)
beta.append(0*pi/180)
for i in range(0, len(T)-1):
	dx=u1[i]*cos(theta[i])
	dy=u1[i]*sin(theta[i])
	dtheta=u1[i]/L*tan(beta[i])
	dbeta=u2[i]
	x.append(x[i]+dx*dt)
	y.append(y[i]+dy*dt)
	theta.append(theta[i]+dtheta*dt)
	beta.append(beta[i]+dbeta*dt)
plt.plot(x,y)

###################"iNitial posture#################"""
x0=0;
y0=-1.5;
theta0=(0*pi/180);
beta0=0;
################################################"

z1=x0-x[0];
z2=y0-y[0];
z3=tan(theta0-theta[0]);
z4=(tan(beta0)-cos(theta0-theta[0])*tan(beta[0]))/(L*cos(theta0-theta[0])**3)+k2*z2;
#######################################################
xt=[]
yt=[]
thetat=[]
betat=[]
u1t=[]
u2t=[]
xt.append(x0)
yt.append(y0)
thetat.append(theta0)
u1t.append(0)
u2t.append(0)
betat.append(atan((z4-k2*z2)*(L*cos(thetat[0]-theta[0])**3)+cos(thetat[0]-theta[0])*tan(beta[0])))

exp=(u1[0]/L*tan(beta[0]))*z2+u1t[0]*cos(atan(z3))-u1[0]
eyp=-(u1[0]/L*tan(beta[0]))*z1+u1t[0]*sin(atan(z3))
ethetap=u1t[0]/L*tan(betat[0])-u1[0]/L*tan(beta[0])
w1=u1t[0]*cos(atan(z3))-u1[0]	
w2=k2*eyp+(  3*tan(betat[0])/cos(atan(z3))-2*tan(beta[0])    )*sin(atan(z3))/(  L*cos(atan(z3))**2  )*ethetap-u2[0]/(  L*cos(beta[0])**2*cos(atan(z3))**2  )+u2t[0]/(  L*cos(betat[0])**2*cos(atan(z3))**2  )
for i in range(0, len(T)-1):
	
	dz1=(u1[i]/L*tan(beta[i]))*z2+w1;
	dz2=-(u1[i]/L*tan(beta[i]))*z1+u1[i]*z3+w1*z3;
	dz3=-k2*u1[i]*z2+u1[i]*z4+w1*(z4-k2*z2+(1+z3**2)*tan(beta[i])/L);	
	dz4=w2;
	z1=z1+dt*dz1;
	z2=z2+dt*dz2;
	z3=z3+dt*dz3;
	z4=z4+dt*dz4;
	w1=-k1*abs(u1[i])*(z1+z3/k2*(z4+(1+z3**2)*tan(beta[i])/L));
	w2=-k3*u1[i]*z3-k4*abs(u1[i])*z4;
	ex=z1
	ey=z2
	etheta=atan(z3)
	u1t.append((w1+u1[i])/cos(etheta))	
	betat.append(atan((z4-k2*ey)*(L*cos(etheta)**3)+cos(etheta)*tan(beta[i])))


for i in range(0, len(T)-1):
	dtheta=u1t[i]/L*tan(betat[i])
	dx=u1t[i]*cos(thetat[i])
	dy=u1t[i]*sin(thetat[i])

	xt.append(xt[i]+dx*dt)
	yt.append(yt[i]+dy*dt)
	thetat.append(thetat[i]+dtheta*dt)
		


plt.plot(xt,yt)
plt.show()


