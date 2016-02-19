#!/usr/bin/env python

#The main objective of this node will be to generate the points for the primitive trajectories

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
tb_pos=[]
tb_ori=[]

###########################################TRAJECTORY 1: ARM TO THE FRONT#########################
#Trajectory 0 -> arm to the front X COORDINATE
#First we define all the x points for that trajectory, from the initial pose.
#Time represents the number of points used to gen the traj
time=np.arange(0,4,1)
time_step=1
pos_x1=[0.0,0.19536, 0.4245, 0.53072]
s_x.append(interpolate.splrep(time,pos_x1,k=3))

#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[0],der=0))

#Trajectory 0 -> arm to the front Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y1=[-0.335, -0.33521, -0.33522, -0.33315]
s_y.append(interpolate.splrep(time,pos_y1,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[0],der=0))


#Trajectory 0 -> arm to the front Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z1=[-0.03531, 0.022999, 0.21556, 0.48317]
s_z.append(interpolate.splrep(time,pos_z1,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[0],der=0))

#Trajectory 0 -> eef orientation X
ori_x1=[0.0, 1.1146e-05, -4.0355e-06, 1.841e-05]
#s_ox.append(interpolate.splrep(time,ori_x1,k=3))
s_ox.append(ori_x1)
#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[0],der=0))


#Trajectory 0 -> eef orientation Y
ori_y1=[0.0, -0.44642, -0.73769, -0.73771]
#s_oy.append(interpolate.splrep(time,ori_y1,k=3))
s_oy.append(ori_y1)
#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[0],der=0))

#Trajectory 0 -> eef orientation Z
ori_z1=[0.0, 1.012e-06, 1.2689e-05, 8.3309e-05]
#s_oz.append(interpolate.splrep(time,ori_z1,k=3))
s_oz.append(ori_z1)
#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[0],der=0))


#Trajectory 0 -> eef orientation w
ori_w1=[1, 0.89482, 0.67514, 0.67512]
#s_ow.append(interpolate.splrep(time,ori_w1,k=3))
s_ow.append(ori_w1)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[0],der=0))

#Feature vector first trajectory
tb_pos.append([pos_x1[-1],pos_y1[-1],pos_z1[-1]])
tb_ori.append([ori_x1[-1],ori_y1[-1],ori_z1[-1],ori_w1[-1]])
###########################################TRAJECTORY 2: ARM UP#########################
#First we define all the x points for that trajectory, from the initial pose.
max_time=4
time=np.arange(0,max_time,1)
pos_x2=[0.0, 0.4245, 0.40463, 0.015251]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x2,k=3))

#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[1],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y2=[-0.34, -0.33522, -0.33586, -0.33577]
s_y.append(interpolate.splrep(time,pos_y2,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[1],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z2=[-0.03531, 0.21556, 0.81458, 1.0207]
s_z.append(interpolate.splrep(time,pos_z2,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[1],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x2=[0.0, -4.0355e-06, 1.9511e-05, 1.0215e-06]
#s_ox.append(interpolate.splrep(time,ori_x2,k=3))
s_ox.append(ori_x2)
#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[1],der=0))


#Trajectory 1 ->arm up eef orientation Y
ori_y2=[0.0, -0.73769, 0.89069, 0.99997]
#s_oy.append(interpolate.splrep(time,ori_y2,k=3))
s_oy.append(ori_y2)
#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[1],der=0))


#Trajectory 1 ->arm up eef orientation Z
ori_z2=[0.0, 1.2689e-05, 2.0518e-05, -2.4374e-08]
#s_oz.append(interpolate.splrep(time,ori_z2,k=3))
s_oz.append(ori_z2)
#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[1],der=0))


#Trajectory 1 ->arm up eef orientation w
ori_w2=[1, 0.67514, -0.4546, 0.0077762]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
s_ow.append(ori_w2)
#print("ESTA ES LA SPLINE!:::")
#print(s_ow)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[1],der=0))

#feature vector trajectory 2
tb_pos.append([pos_x2[-1],pos_y2[-1],pos_z2[-1]])
tb_ori.append([ori_x2[-1],ori_y2[-1],ori_z2[-1],ori_w2[-1]])

'''
############################################TRAJECTORY 3: Brazo diagonal x########################
#First we define all the x points for that trajectory, from the initial pose.
max_time=4
time=np.arange(0,max_time,1)
pos_x3=[0.0 ,0.31722, 0.4085, 0.2729]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x3,k=3))
#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[2],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y3=[-0.34, -0.43885, -0.4861,-0.74706]
s_y.append(interpolate.splrep(time,pos_y3,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[2],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z3=[-0.03531, 0.083367, 0.19949,0.36594]
s_z.append(interpolate.splrep(time,pos_z3,k=3))
#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[2],der=0))
#Trajectory 1 -> arm up eef orientation X
ori_x3=[0.0, -0.093019, -0.055323,-0.32561]
#s_ox.append(interpolate.splrep(time,ori_x2,k=3))
s_ox.append(ori_x3)
#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[1],der=0))


#Trajectory 1 ->arm up eef orientation Y
ori_y3=[0.0, -0.3533, -0.44661, -0.58317]
#s_oy.append(interpolate.splrep(time,ori_y2,k=3))
s_oy.append(ori_y3)
#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[1],der=0))

#Trajectory 1 ->arm up eef orientation Z
ori_z3=[0.0, -0.054784, -0.16761, -0.63013]
#s_oz.append(interpolate.splrep(time,ori_z2,k=3))
s_oz.append(ori_z3)
#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[1],der=0))


#Trajectory 1 ->arm up eef orientation w
ori_w3=[1, 0.92926, 0.87714, 0.622]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
s_ow.append(ori_w3)
#print("ESTA ES LA SPLINE!:::")
#print(s_ow)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[1],der=0))

#feature vector trajectory 3
tb_pos.append([pos_x3[-1],pos_y3[-1],pos_z3[-1]])
tb_ori.append([ori_x3[-1],ori_y3[-1],ori_z3[-1],ori_w3[-1]])

###########################################TRAJECTORY 4: Brazo al frete complicada#########################
#First we define all the x points for that trajectory, from the initial pose.
max_time=4
time=np.arange(0,max_time,1)
pos_x4=[0.0, 0.20067, 0.43917, 0.51766]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x4,k=3))

#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[2],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y4=[-0.34, -0.33913, -0.30406, -0.32059]
s_y.append(interpolate.splrep(time,pos_y4,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[2],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z4=[-0.03531, 0.20267, 0.386, 0.60252]
s_z.append(interpolate.splrep(time,pos_z4,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[2],der=0))
#Trajectory 1 -> arm up eef orientation X
ori_x4=[0.0, 0.015235, -2.9376e-05, -1.4106e-05]
#s_ox.append(interpolate.splrep(time,ori_x2,k=3))
s_ox.append(ori_x4)
#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[1],der=0))


#Trajectory 1 ->arm up eef orientation Y
ori_y4=[0.0, -0.77353, 0.87459, -0.80963]
#s_oy.append(interpolate.splrep(time,ori_y2,k=3))
s_oy.append(ori_y4)
#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[1],der=0))


#Trajectory 1 ->arm up eef orientation Z
ori_z4=[0.0, 0.023811, -2.5394e-05, -6.5469e-06]
#s_oz.append(interpolate.splrep(time,ori_z2,k=3))
s_oz.append(ori_z4)
#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[1],der=0))


#Trajectory 1 ->arm up eef orientation w
ori_w4=[1, 0.63312, -0.48487, 0.58694]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
s_ow.append(ori_w4)
#print("ESTA ES LA SPLINE!:::")
#print(s_ow)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[1],der=0))

#feature vector trajectory 4
tb_pos.append([pos_x4[-1],pos_y4[-1],pos_z4[-1]])
tb_ori.append([ori_x4[-1],ori_y4[-1],ori_z4[-1],ori_w4[-1]])

'''
'''
###########################################TRAJECTORY 5: Solo brazo al frete, pos init distinta#########################
#First we define all the x points for that trajectory, from the initial pose.
max_time=4
time=np.arange(0,max_time,1)
pos_x5=[0.38,0.41336, 0.47169, 0.52912]
#Trajectory 1 -> arm up X COORDINATE 
s_x.append(interpolate.splrep(time,pos_x5,k=3))

#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[2],der=0))

#Trajectory 1 -> arm up Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y5=[-0.35301, -0.353, -0.353, -0.35296]
s_y.append(interpolate.splrep(time,pos_y5,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[2],der=0))


#Trajectory 1 -> arm up Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z5=[0.48719, 0.48824, 0.49074, 0.4932]
s_z.append(interpolate.splrep(time,pos_z5,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[2],der=0))

#Trajectory 1 -> arm up eef orientation X
ori_x5=[-6.0772e-05, 5.9131e-05, -3.389e-05, 0.00010115]
#s_ox.append(interpolate.splrep(time,ori_x2,k=3))
s_ox.append(ori_x5)
#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[1],der=0))


#Trajectory 1 ->arm up eef orientation Y
ori_y5=[-0.72207, -0.72196, -0.72202, -0.72199]
#s_oy.append(interpolate.splrep(time,ori_y2,k=3))
s_oy.append(ori_y5)
#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[1],der=0))


#Trajectory 1 ->arm up eef orientation Z
ori_z5=[4.1785e-05, -2.8837e-05, 2.5799e-05, -2.6761e-05]
#s_oz.append(interpolate.splrep(time,ori_z2,k=3))
s_oz.append(ori_z5)
#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[1],der=0))


#Trajectory 1 ->arm up eef orientation w
ori_w5=[0.69182,0.69193, 0.69187, 0.6919]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
s_ow.append(ori_w5)
#print("ESTA ES LA SPLINE!:::")
#print(s_ow)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[1],der=0))

#feature vector trajectory5
tb_pos.append([pos_x5[-1],pos_y5[-1],pos_z5[-1]])
tb_ori.append([ori_x5[-1],ori_y5[-1],ori_z5[-1],ori_w5[-1]])
'''
##################################################################################################################
#plt.figure()
#plt.plot(time,pos_x3)
#plt.axis([0, 3, -1.5, 1.5])
#plt.title('X trajectory spline interpolation')

#plt.figure()
#plt.plot(time,pos_y3)
#plt.axis([0, 3, -1.5, 1.5])
#plt.title('Y trajectory spline interpolation')

#plt.figure()
#plt.plot(time,pos_z3)
#plt.axis([0, 3, -1.5, 1.5])
#plt.title('Z trajectory spline interpolation')
#plt.show()


'''
#First we define all the x points for that tra	jectory, from the initial pose.

pos_x3=[0.0, -0.0058587, 0.40463, 0.015251]

s_x.append(interpolate.splrep(time,pos_x2,k=3))

#We define an x array to evaluate the spline (plot rep).
time_spline=np.arange(0,3.1,0.1)
pos_x.append(interpolate.splev(time_spline,s_x[1],der=0))

#plt.figure()
#plt.plot(time,pos,'o', time_spline,pos_spline)
#plt.axis([0, 3.5, -1, 1])
#plt.title('X trajectory spline interpolation')
#plt.show()

#Trajectory 1 -> arm to the front Y COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_y2=[-0.34, -0.61833, -0.33586, -0.33577]
s_y.append(interpolate.splrep(time,pos_y2,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_y.append(interpolate.splev(time_spline,s_y[1],der=0))


#Trajectory 1 -> arm to the front Z COORDINATE
#First we define all the points for that trajectory, from the initial pose.
pos_z2=[-0.03531, 0.050813, 0.81458, 1.0207]
s_z.append(interpolate.splrep(time,pos_z2,k=3))

#We define an x array to evaluate the spline (plot rep).
pos_z.append(interpolate.splev(time_spline,s_z[1],der=0))

#Trajectory 1 -> eef orientation X
ori_x2=[0.0, -0.29236, 1.9511e-05, 1.0215e-06]
#s_ox.append(interpolate.splrep(time,ori_x2,k=3))

#We define an x array to evaluate the spline (plot rep).
#ori_x.append(interpolate.splev(time_spline,s_ox[1],der=0))


#Trajectory 1 -> eef orientation Y
ori_y2=[0.0, 1.3847e-05, 0.89069, 0.99997]
#s_oy.append(interpolate.splrep(time,ori_y2,k=3))

#We define an x array to evaluate the spline (plot rep).
#ori_y.append(interpolate.splev(time_spline,s_oy[1],der=0))

#Trajectory 1 -> eef orientation Z
ori_z2=[0.0, -2.7526e-05, 2.0518e-05, -2.4374e-08]
#s_oz.append(interpolate.splrep(time,ori_z2,k=3))

#We define an x array to evaluate the spline (plot rep).
#ori_z.append(interpolate.splev(time_spline,s_oz[1],der=0))


#Trajectory 1 -> eef orientation w
ori_w2=[1, 0.95631, -0.4546, 0.0077762]
#s_ow.append(interpolate.splrep(time,ori_w2,k=3))
#print("ESTA ES LA SPLINE!:::")
#print(s_ow)
#We define an x array to evaluate the spline (plot rep).
#ori_w.append(interpolate.splev(time_spline,s_ow[1],der=0))
'''
