# -*- coding: utf-8 -*-
"""
Created on Mon Apr 13 21:54:44 2020
ECE470 Lab5 file

@author: Marshill
"""
import numpy as np
import math

L1 = 0.152
L2 = 0.12
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.083
L7 = 0.083
L8 = 0.082
L9 = 0.0535
L10 = 0.059



#world frame coordinate
Xw = 0.15;
Yw = -0.1;
Zw = 0.25;
yaw = -np.pi/4;

#robot arm centered coordinate
Xgrip = Xw + 0.15
Ygrip = Yw - 0.15
Zgrip = Zw - 0.01

#cen coordinate
Xcen = Xgrip-L9*np.cos(yaw)
Ycen = Ygrip-L9*np.sin(yaw)
Zcen = Zgrip

p = L2-L4+L6
r = np.sqrt(Xcen**2+Ycen**2)
theta1 = math.atan2(Ycen,Xcen)-math.atan2(p,np.sqrt(r**2-p**2))

theta6 = theta1+np.pi/2-yaw

#3end coordinate
X3end = Xcen-L7*np.cos(theta1)+(L6+0.027)*np.sin(theta1)
Y3end = Ycen-L7*np.sin(theta1)-(L6+0.027)*np.cos(theta1)
Z3end = Zcen+L10+L8

Lt = np.sqrt(X3end**2+Y3end**2+(Z3end-L1)**2)
theta3 = np.pi-np.arccos((L3**2+L5**2-Lt**2)/(2*L3*L5))
theta2 = -np.arccos((L3**2+Lt**2-L5**2)/(2*L3*Lt))-math.atan2(Z3end-L1,np.sqrt(X3end**2+Y3end**2))
theta4 = -theta2-theta3
theta5 = -np.pi/2

print("Thetas:"," ",theta1/np.pi*180," ",theta2/np.pi*180," ",theta3/np.pi*180," ",theta4/np.pi*180," ",theta5/np.pi*180," ",theta6/np.pi*180)
print("Thetas(in rad):"," ",theta1," ",theta2," ",theta3," ",theta4," ",theta5," ",theta6)




