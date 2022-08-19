#!/usr/bin/env python

import numpy as np

import transformation



rotation_mat = transformation.rotation_matrix(10, 20, 30, 'xyz')

print(rotation_mat)
# [[ 0.92541658 -0.20487413  0.31879578]
#  [ 0.34202014  0.81379768 -0.46984631]
#  [-0.16317591  0.54383814  0.82317294]]

print "sub matrix"
matrixq = np.array([[1, 0, 0, 8],[0, 1, 0,9],[0,0,1,7]])


print matrixq

sub_matriz=matrixq[:,0:3]
print ("sub matrix")

print(sub_matriz)

angles = transformation.rotation_angles(sub_matriz, 'xyz')


print "angulos"
#Calculate Euler angles of YZX order from the rotation matrix

print(angles)
# (10.0, 20.0, 29.999999999999993)
#