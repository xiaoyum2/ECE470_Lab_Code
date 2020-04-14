#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
#from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
PI = 180
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	
	# find M and S by measuring the UR3 robot arm
    # those values are calculated using MATLAB, file is uploaded as well
	M = np.array([[0.,-1.,0.,0.39],[0.,0.,-1.,0.401],[1.,0.,0.,0.2155], [0.,0.,0.,1.]])
	S = np.array([[0.,0.,0.,0.,1.,0.], [0.,1.,1.,1.,0.,1.],[1.,0.,0.,0.,0.,0.],[0.150000000000000,-0.162000000000000,-0.162000000000000,-0.162000000000000,0.,-0.162000000000000],[0.150000000000000,0,0,0,0.162000000000000,0],[0,-0.150000000000000,0.0940000000000000,0.307000000000000,-0.260000000000000,0.390000000000000]])

	
	# ==============================================================#
	return M, S



"""
Function that calculates encoder numbers for each motor
input thetas in degree
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	#theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

    #calculated the matrix-fied s
	s1_mat = np.array([[0, -S[2][0], S[1][0], S[3][0]],[S[2][0], 0, -S[0][0], S[4][0]], [-S[1][0], S[0][0], 0, S[5][0]], [0,0,0,0]])
	s2_mat = np.array([[0, -S[2][1], S[1][1], S[3][1]],[S[2][1], 0, -S[0][1], S[4][1]], [-S[1][1], S[0][1], 0, S[5][1]], [0,0,0,0]])
	s3_mat = np.array([[0, -S[2][2], S[1][2], S[3][2]],[S[2][2], 0, -S[0][2], S[4][2]], [-S[1][2], S[0][2], 0, S[5][2]], [0,0,0,0]])
	s4_mat = np.array([[0, -S[2][3], S[1][3], S[3][3]],[S[2][3], 0, -S[0][3], S[4][3]], [-S[1][3], S[0][3], 0, S[5][3]], [0,0,0,0]])
	s5_mat = np.array([[0, -S[2][4], S[1][4], S[3][4]],[S[2][4], 0, -S[0][4], S[4][4]], [-S[1][4], S[0][4], 0, S[5][4]], [0,0,0,0]])
	s6_mat = np.array([[0, -S[2][5], S[1][5], S[3][5]],[S[2][5], 0, -S[0][5], S[4][5]], [-S[1][5], S[0][5], 0, S[5][5]], [0,0,0,0]])

    #equations for calculating T from s and theta
	T = expm(s1_mat*(theta1*np.pi/180)).dot(expm(s2_mat*(theta2*np.pi/180))).dot(expm(s3_mat*(theta3*np.pi/180))).dot(expm(s4_mat*(theta4*np.pi/180))).dot(expm(s5_mat*(theta5*np.pi/180))).dot(expm(s6_mat*(theta6*np.pi/180))).dot(M)



	# ==============================================================#
	
	print(str(T) + "\n"+str(T[0][3])+" "+str(T[1][3])+" "+str(T[2][3]))
    
    
	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


lab_fk(-58.01801824645075 ,  -84.5256538707446 ,  88.26386091279613 ,  -3.738207042051531 ,  -90.0 ,  76.98198175354925)

