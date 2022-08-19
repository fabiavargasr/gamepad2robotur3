#!/usr/bin/env python

import numpy as np

import transformation



print "sub matrix"
matrixq = np.array([[1, 0, 0, 8],[0, 1, 0,9],[0,0,1,7]])


print matrixq

sub_matriz=matrixq[:,0:3]
print ("sub matrix")

print(sub_matriz)

angles = transformation.rotation_angles(sub_matriz, 'xyz')

print "angulos"
print(angles)

tx=angles[0]+0.2
ty=angles[1]+0.51
tz=angles[2]+0.8


print ("angulos 22",tx, ty, tz)


rotation_mat = transformation.rotation_matrix(tx, ty, tz, 'xyz')
print "reacion nueva ocons"
print(rotation_mat)

position = np.zeros((3, 1))

position[0]=32
position[1]=24
position[2]=652

nueva_mat = np.c_[rotation_mat, position]

print(nueva_mat)







