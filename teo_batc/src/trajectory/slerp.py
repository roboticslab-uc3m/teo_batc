#!/usr/bin/env python  
# -*- encoding: utf-8 -*-
'''
This code part of the project "New task generation for humanoid robots based on case and user communication"

Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)

This is the python code to generate de SLERP solution for two different vectors.
SLERP stands for spherical linear interpolation, we use this to calculate the 
new quaternion (quaternions are not positions in the space), so linear
interpolation just dont work
'''
from numpy import *
import numpy as np
from numpy.linalg import norm

def interp(q1, q2, r):
    if np.dot(q1,q2) < 0: #If dot product is negative we have to negate one quat. to always take the short path
    	q1=np.negative(q1)
    theta = arccos(dot(q1/norm(q1), q2/norm(q2)))
    q = []
    if r<0 or r>1:
        raise 'R out of range'
    if theta == 0:
        q = q1
    else:
        so = sin(theta)
        q.append(sin((1.0-r)*theta)/so*q1[0]+sin(r*theta)/so*q2[0])
        q.append(sin((1.0-r)*theta)/so*q1[1]+sin(r*theta)/so*q2[1])
        q.append(sin((1.0-r)*theta)/so*q1[2]+sin(r*theta)/so*q2[2])
        q.append(sin((1.0-r)*theta)/so*q1[3]+sin(r*theta)/so*q2[3])
        print(q)
        #CARE MAYBE q NEEDS TO BE NORMALIZED
    return q


