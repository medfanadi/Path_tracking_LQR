#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from math import *
import numpy as np
import matplotlib.pyplot as plt
import my_ode45 as myint
k1=1;
k2=1;
k3=3;
k4=3;
L=1.2;
dt=0.001;
T=np.arange(0, 30+dt,dt)#0:dt:30;
u1=[]
u2=[]
for i in range(0, len(T)):
	u1.append(2)
	u2.append(0)
	

x=[]
y=[]
theta=[]
beta=[]
x.append(0)
y.append(0)
theta.append(45*pi/180)
beta.append(0)
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


x0=0;
y0=-1.5;
theta0=(-15*pi/180);
beta0=0;

z1=x0-x[0];
z2=y0-y[0];
z3=tan(theta0-theta[0]);
z4=(tan(beta0)-cos(theta0-theta[0])*tan(beta[0]))/(L*cos(theta0-theta[0])**3)+k2*z2;
xt=[]
yt=[]
thetat=[]
betat=[]
#xt.append(x0)
#yt.append(y0)
#thetat.append(theta0)
#betat.append(beta0)
for i in range(0, len(T)-1):
	xt.append(z1+x[i])
	yt.append(z2+y[i])
	thetat.append(atan(z3)+theta[i])
	betat.append(atan((z4-k2*z2)*(L*cos(thetat[i]-theta[i])**3)+cos(thetat[i]-theta[i])*tan(beta[i])))
	w1=-k1*abs(u1[i])*(z1+z3/k2*(z4+(1+z3**2)*tan(beta[i])/L));
	w2=-k3*u1[i]*z3-k4*abs(u1[i])*z4;
	dz1=(u1[i]/L*tan(beta[i]))*z2+w1;
	dz2=-(u1[i]/L*tan(beta[i]))*z1+u1[i]*z3+w1*z3;
	dz3=-k2*u1[i]*z2+u1[i]*z4+w1*(z4-k2*z2+(1+z3**2)*tan(beta[i])/L);	
	dz4=w2;
	z1=z1+dt*dz1;
	z2=z2+dt*dz2;
	z3=z3+dt*dz3;
	z4=z4+dt*dz4;


xt.append(z1+x[i+1]);
yt.append(z2+y[i+1]);


thetat.append(atan(z3)+theta[i+1]);
betat.append(atan((z4-k2*z2)*(L*cos(thetat[i+1]-theta[i+1])**3)+cos(thetat[i+1]-theta[i+1])*tan(beta[i+1])));
plt.plot(xt,yt)
plt.show()





