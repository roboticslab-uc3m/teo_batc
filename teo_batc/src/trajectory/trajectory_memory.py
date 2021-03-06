#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
This code part of the project "New task generation for humanoid robots based on case and user communication"

Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
This node, is the one in charge to act, like the database
of the system. Here all the tasks are stored, with the characteristics that
defines them, and that will be used for the indexing problem. This means using different
arrays of two dimensions to store things, such as, the characteristics of the
trajectory, or the splines values that defines it.

'''
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

########################################################SPLINE TRAJECTORIES###########################################################


#Init s lists this arrays contains the splines trajectories for each traj.
s_x=[]
s_y=[]
s_z=[]
s_ox=[]
s_oy=[]
s_oz=[]
s_ow=[]

#Init splev list
pos_x=[]
pos_y=[]
pos_z=[]
ori_x=[]
ori_y=[]
ori_z=[]
ori_w=[]

#Init characteristic array
feature=[]

###########################################TRAJECTORY 1: ARM TO THE FRONT#########################
#Trajectory 0 -> arm to the front X COORDINATE
#First we define all the x points for that trajectory, from the initial pose.
#Time represents the number of points used to gen the traj
time=np.arange(0,4,1)
time_step=1
pos_x1=[0.0, 0.19536, 0.4245, 0.53072]
s_x.append(interpolate.splrep(time,pos_x1,k=3))
#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1, 0.1)
pos_x.append(interpolate.splev(time_spline,s_x[0],der=0))

#Trajectory 0 -> arm to the front Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y1=[-0.335, -0.33521, -0.33522, -0.33315]
s_y.append(interpolate.splrep(time,pos_y1,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[0],der=0))


#Trajectory 0 -> arm to the front Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z1=[-0.03531, 0.022999, 0.21556, 0.48317]
s_z.append(interpolate.splrep(time,pos_z1,k=3))
#We define an z array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[0],der=0))

#Trajectory 0 -> eef orientation X
ori_x1=[0.0, 1.1146e-05, -4.0355e-06, 1.841e-05]
s_ox.append(ori_x1)

#Trajectory 0 -> eef orientation Y
ori_y1=[0.0, -0.44642, -0.73769, -0.73771]
s_oy.append(ori_y1)

#Trajectory 0 -> eef orientation Z
ori_z1=[0.0, 1.012e-06, 1.2689e-05, 8.3309e-05]
s_oz.append(ori_z1)

#Trajectory 0 -> eef orientation w
ori_w1=[1, 0.89482, 0.67514, 0.67512]
s_ow.append(ori_w1)

#Feature vector first trajectory
feature.append([pos_x1[-1],pos_y1[-1],pos_z1[-1],ori_x1[-1],ori_y1[-1],ori_z1[-1],ori_w1[-1],0,0,0,0,0,0,0])

###########################################TRAJECTORY 2: ARM UP#########################
#First we define all the x points for that trajectory, from the initial pose.
pos_x2=[0.0, 0.4245, 0.40463, 0.015251]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x2,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_x.append(interpolate.splev(time_spline,s_x[1],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y2=[-0.34, -0.33522, -0.33586, -0.33577]
s_y.append(interpolate.splrep(time,pos_y2,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[1],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z2=[-0.03531, 0.21556, 0.81458, 1.0207]
s_z.append(interpolate.splrep(time,pos_z2,k=3))
#We define an z array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[1],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x2=[0.0, -4.0355e-06, 1.9511e-05, 1.0215e-06]
s_ox.append(ori_x2)

#Trajectory 1 ->arm up eef orientation Y
ori_y2=[0.0, -0.73769, 0.89069, 0.99997]
s_oy.append(ori_y2)

#Trajectory 1 ->arm up eef orientation Z
ori_z2=[0.0, 1.2689e-05, 2.0518e-05, -2.4374e-08]
s_oz.append(ori_z2)

#Trajectory 1 ->arm up eef orientation w
ori_w2=[1, 0.67514, -0.4546, 0.0077762]
s_ow.append(ori_w2)

#feature vector trajectory 2
feature.append([pos_x2[-1],pos_y2[-1],pos_z2[-1],ori_x2[-1],ori_y2[-1],ori_z2[-1],ori_w2[-1],0,0,0,0,0,0,0])

#Uncomment the next section for adding more trajectories to the database and study how the system behaves
'''

############################################TRAJECTORY 3: frente atras########################

#First we define all the x points for that trajectory, from the initial pose.
pos_x6=[0.0 ,-0.30785, -0.48385, -0.52916]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x6,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_x.append(interpolate.splev(time_spline,s_x[2],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y6=[-0.34, -0.32636,-0.31563,-0.31562]
s_y.append(interpolate.splrep(time,pos_y6,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[2],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z6=[-0.03531, 0.063911, 0.28383,0.49877]
s_z.append(interpolate.splrep(time,pos_z6,k=3))
#We define an z array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[2],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x6=[0.0, 4.3542e-06,-2.3967e-05,-5.8599e-05]
s_ox.append(ori_x6)



#Trajectory 1 ->arm up eef orientation Y
ori_y6=[0.0, 0.34296, 0.55659, 0.6982]
s_oy.append(ori_y6)

#Trajectory 1 ->arm up eef orientation Z
ori_z6=[0.0, -6.3779e-06, 2.5766e-05, -1.1143e-05]
s_oz.append(ori_z6)


#Trajectory 1 ->arm up eef orientation w
ori_w6=[1, 0.93935, 0.83079, 0.7159]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
s_ow.append(ori_w6)

#feature vector trajectory 6
feature.append([pos_x6[-1],pos_y6[-1],pos_z6[-1],ori_x6[-1],ori_y6[-1],ori_z6[-1],ori_w6[-1],0,0,0,0,0,0,0])


############################################TRAJECTORY 3: Brazo diagonal x########################
#First we define all the x points for that trajectory, from the initial pose.
pos_x3=[0.0 ,0.31722, 0.4085, 0.2729]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x3,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_x.append(interpolate.splev(time_spline,s_x[2],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y3=[-0.34, -0.43885, -0.4861,-0.74706]
s_y.append(interpolate.splrep(time,pos_y3,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[2],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z3=[-0.03531, 0.083367, 0.19949,0.36594]
s_z.append(interpolate.splrep(time,pos_z3,k=3))
#We define an z array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[2],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x3=[0.0, -0.093019, -0.055323,-0.32561]
s_ox.append(ori_x3)


#Trajectory 1 ->arm up eef orientation Y
ori_y3=[0.0, -0.3533, -0.44661, -0.58317]
s_oy.append(ori_y3)


#Trajectory 1 ->arm up eef orientation Z
ori_z3=[0.0, -0.054784, -0.16761, -0.63013]
s_oz.append(ori_z3)

#Trajectory 1 ->arm up eef orientation w
ori_w3=[1, 0.92926, 0.87714, 0.622]
s_ow.append(ori_w3)

#feature vector trajectory 3
feature.append([pos_x3[-1],pos_y3[-1],pos_z3[-1],ori_x3[-1],ori_y3[-1],ori_z3[-1],ori_w3[-1],0,0,0,0,0,0,0])
'''

'''
###########################################TRAJECTORY 4: Brazo al frete complicada#########################
#First we define all the x points for that trajectory, from the initial pose.
pos_x4=[0.0, 0.20067, 0.43917, 0.51766]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x4,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_x.append(interpolate.splev(time_spline,s_x[3],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y4=[-0.34, -0.33913, -0.30406, -0.32059]
s_y.append(interpolate.splrep(time,pos_y4,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[3],der=0))

#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z4=[-0.03531, 0.20267, 0.386, 0.60252]
s_z.append(interpolate.splrep(time,pos_z4,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[3],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x4=[0.0, 0.015235, -2.9376e-05, -1.4106e-05]
s_ox.append(ori_x4)

#Trajectory 1 ->arm up eef orientation Y
ori_y4=[0.0, -0.77353, 0.87459, -0.80963]
s_oy.append(ori_y4)

#Trajectory 1 ->arm up eef orientation Z
ori_z4=[0.0, 0.023811, -2.5394e-05, -6.5469e-06]
s_oz.append(ori_z4)

#Trajectory 1 ->arm up eef orientation w
ori_w4=[1, 0.63312, -0.48487, 0.58694]
s_ow.append(ori_w4)

#feature vector trajectory 4
feature.append([pos_x4[-1],pos_y4[-1],pos_z4[-1],ori_x4[-1],ori_y4[-1],ori_z4[-1],ori_w4[-1],0,0,0,0,0,0,0])

###########################################TRAJECTORY 5: Solo brazo al frete, pos init distinta#########################
#First we define all the x points for that trajectory, from the initial pose.
pos_x5=[0.38,0.41336, 0.47169, 0.52912]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x5,k=3))
#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[4],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y5=[-0.35301, -0.353, -0.353, -0.35296]
s_y.append(interpolate.splrep(time,pos_y5,k=3))
#We define an y array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[4],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z5=[0.48719, 0.48824, 0.49074, 0.4932]
s_z.append(interpolate.splrep(time,pos_z5,k=3))
#We define an z array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[4],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x5=[-6.0772e-05, 5.9131e-05, -3.389e-05, 0.00010115]
s_ox.append(ori_x5)

#Trajectory 1 ->arm up eef orientation Y
ori_y5=[-0.72207, -0.72196, -0.72202, -0.72199]
s_oy.append(ori_y5)

#Trajectory 1 ->arm up eef orientation Z
ori_z5=[4.1785e-05, -2.8837e-05, 2.5799e-05, -2.6761e-05]
s_oz.append(ori_z5)

#Trajectory 1 ->arm up eef orientation w
ori_w5=[0.69182,0.69193, 0.69187, 0.6919]
s_ow.append(ori_w5)

#feature vector trajectory5
feature.append([pos_x5[-1],pos_y5[-1],pos_z5[-1],ori_x5[-1],ori_y5[-1],ori_z5[-1],ori_w5[-1],0,0,0,0,0,0,0])
'''
