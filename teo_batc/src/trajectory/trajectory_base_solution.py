#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
This code part of the project "New task generation for humanoid robots based on case and user communication"

Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)

#The main objetive of this code is to generate a base solution with the
#trajectories in the memory robot. First, the two best trajecto-
ries of the database, are selected, and then the weights assigned to it, are
calculated, using for that the final trajectory information. To finish these
weights, are returned, to the caller function.
'''
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import math
from operator import sub, add
from scipy import interpolate
from trajectory_memory import *

def base_trajectory_gen(tf):
    #First we define the feature array of th trajectories
    #The feature array structurie [final_pos,intermediate_req(x_const?,y_const?...]
    print("Estoy dentro de la funcion b_tr_gen")
    #First we choose the 2 trajectories closer to the final solution
    count=0
    b_dis=[10000,10000]
    b_s=[1,2]
    #this while goes through all the trajectories we have
    while count<len(feature):
        feature_sum=map(sub,tf,feature[count])
        dis=math.sqrt(sum([x**2 for x in feature_sum])) #Euclidian distance
        #print(dis)
        if dis<b_dis[0]:
            #Move the firs sol to the second pos
            b_s[1]=b_s[0]
            b_dis[1]=b_dis[0]
            #Add the first sol to the first pos
            b_s[0]=count
            b_dis[0]=dis
        elif dis<b_dis[1]:
            b_s[1]=count
            b_dis[1]=dis
        count+=1
    #Now we calculate the weights for each pos coordinate
    w1=[]
    w2=[]
    for j in np.arange(0,3,1):
        dis1=math.sqrt((feature[b_s[0]][j]-tf[j])**2)
        dis2=math.sqrt((feature[b_s[1]][j]-tf[j])**2)
        w1.append(dis2/(dis1+dis2))
        w2.append(dis1/(dis1+dis2))

    #In b_s[1] and b_s[2] we have the id of the trajectories in memory
    #Now we will generate the base solution as a wighted sum of the twotrajectories
    w1.append(b_dis[0]/(b_dis[0]+b_dis[1]))
    w2.append(b_dis[1]/(b_dis[0]+b_dis[1]))
    print("las trayectorias cogidas como base son:",b_s[0], b_s[1])
    
    #time=np.arange(0,4,1)
    #plt.figure()
    #plt.plot(time,pos_x1,'o',time, pos_x2, 'x', time_spline,pos_xf)
    #plt.axis([0, 3.5, -1, 1])
    #plt.title('X trajectory spline interpolation')
    #plt.show()
    #We return the 2 trajectories and the weights that define our traj
    return(b_s,w1,w2)
