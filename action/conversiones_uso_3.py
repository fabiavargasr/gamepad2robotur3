#!/usr/bin/env python

import numpy as np

import transformation



print "matrix"
matrixq = np.array([[1, 0, 0, 8],[0, 1, 0,9],[0,0,1,7]])

sub_matriz=matrixq[:,0:3]
angles = transformation.rotation_angles(sub_matriz, 'xyz')
tx=angles[0]+0.2
ty=angles[1]+0.51
tz=angles[2]+0.8
rotation_mat = transformation.rotation_matrix(tx, ty, tz, 'xyz')


position = np.zeros((3, 1))
position[0]=32
position[1]=24
position[2]=652
nueva_mat = np.c_[rotation_mat, position]

print(nueva_mat)







