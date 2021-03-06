#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
This code part of the project "New task generation for humanoid robots based on case and user communication"

Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)

This is the central node in the trajectory block, it is in
charge of the startup, control, and communication, of the two other parts
of the system. Here is where the user defines the final desired trajectory,
that will be send to the ”trajectory base solution”, and where the weights,
returned by this node, are used to perform the weighted sum. Finally, this
information will be send to the ”central node”, to be executed in the sim-
ulation.
'''
import rospy
from teo_batc.srv import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from trajectory_base_solution import *
from slerp import *

print("waiting for service")
rospy.wait_for_service('move_group_srv')
move_group_client = rospy.ServiceProxy('move_group_srv', move_group_srv)

def trajectory_gen(req):
    #If service ask me for a primitive, just exe or send primitive
    print("entre dentro del servicio")
    print req.i[0]
    if req.i[0]==0 or req.i[0]==1:
        p=PoseStamped()
        p.header.frame_id = "base_link"
        group_name= "right_arm"
        pos_tolerance= 0.3
        ang_tolerance= 0.3
        t_inc=0.1
        for t in np.arange(0,3,t_inc):
            p.pose.position.x = interpolate.splev(t,s_x[req.i[0]],der=0)
            p.pose.position.y = interpolate.splev(t,s_y[req.i[0]],der=0)
            p.pose.position.z = interpolate.splev(t,s_z[req.i[0]],der=0)
	    #TODO: TEST this mode
            #First we get what two points we need to interpolate. "step" will be the  
            #interval where the loop is, for example step=3 means that it is
            #between the states 3-4
            step=t//time_step
            #print("Next step is")
            step=int(step)
            #print(s_ox[b_s[0]][step])
            #print(s_ox[b_s[0]][step+1])
            #First we interpolate in time
            p10=[s_ox[b_s[0]][step],s_oy[b_s[0]][step],s_oz[b_s[0]][step],s_ow[b_s[0]][step]]
            p11=[s_ox[b_s[0]][step+1],s_oy[b_s[0]][step+1],s_oz[b_s[0]][step+1],s_ow[b_s[0]][step+1]]
            p1=interp(p10,p11,t-step)
	    p.pose.orientation.x=p1[0];
            p.pose.orientation.y=p2[1];
            p.pose.orientation.z=p3[2];
            p.pose.orientation.w=p4[3];
            #We call the service
            move_group_client(group_name, p, pos_tolerance, ang_tolerance)
            #print(p)
    else:
        print("First, lets generate the new trajectory")
        #First we call the base solution generation
     	#Define the list where we gonna save the trajectory
        tf=[0.40788, -0.34028, 0.83533, 6.0216e-05, 0.90002, -6.8045e-05,
                -0.43585, 0,0,0,0,0,0,0]
        pos_xf=[]
        pos_yf=[]
        pos_zf=[]
        ori_xf=[]
        ori_yf=[]
        ori_zf=[]
        ori_wf=[]
        (b_s,w1,w2)=base_trajectory_gen(tf)
        p=PoseStamped()
        p.header.frame_id = "base_link"
        group_name= "right_arm"
        pos_tolerance= 0.35
        ang_tolerance= 0.35
        count=0
        t_inc=0.1
        for t in np.arange(0,3+t_inc,t_inc):
            #Weghted sum
            p.pose.position.x= interpolate.splev(t,s_x[b_s[0]],der=0)*w1[0]+interpolate.splev(t,s_x[b_s[1]],der=0)*w2[0]
            p.pose.position.y= interpolate.splev(t,s_y[b_s[0]],der=0)*w1[1]+interpolate.splev(t,s_y[b_s[1]],der=0)*w2[1]
            p.pose.position.z= interpolate.splev(t,s_z[b_s[0]],der=0)*w1[2]+interpolate.splev(t,s_z[b_s[1]],der=0)*w2[2]
            
            '''
	    In the next step we use the funcion slerp to calculate the new quaternion
            We get the orientation trajectories for the instant "i"
            and the base trajectories b_s[0]
            and b_s[1].
            The first step to do is to interpolate the different points
            extracted from the trajectories 
            '''
            #If we are in the last step the orienation is the same final state
            if t==3:
                p1=[s_ox[b_s[0]][-1],s_oy[b_s[0]][-1],s_oz[b_s[0]][-1],s_ow[b_s[0]][-1]]
                p2=[s_ox[b_s[1]][-1],s_oy[b_s[1]][-1],s_oz[b_s[1]][-1],s_ow[b_s[1]][-1]]
            else:
                #First we get what two points we need to interpolate. "step" will be the  
                #interval where the loop is, for example step=3 means that it is
                #between the states 3-4
                step=t//time_step
                #print("el paso es")
                step=int(step)
                #print(s_ox[b_s[0]][step])
                #print(s_ox[b_s[0]][step+1])
                #First we interpolate in time
                p10=[s_ox[b_s[0]][step],s_oy[b_s[0]][step],s_oz[b_s[0]][step],s_ow[b_s[0]][step]]
                p11=[s_ox[b_s[0]][step+1],s_oy[b_s[0]][step+1],s_oz[b_s[0]][step+1],s_ow[b_s[0]][step+1]]
                p1=interp(p10,p11,t-step)
                p20=[s_ox[b_s[1]][step],s_oy[b_s[1]][step],s_oz[b_s[1]][step],s_ow[b_s[1]][step]]
                p21=[s_ox[b_s[1]][step+1],s_oy[b_s[1]][step+1],s_oz[b_s[1]][step+1],s_ow[b_s[1]][step+1]]
                p2=interp(p20,p21,t-step)
                #Now we interpolate using the weight assigned to each trajectory
                p_ori=interp(p1,p2,w1[3])

                p.pose.orientation.x=p_ori[0];
                p.pose.orientation.y=p_ori[1];
                p.pose.orientation.z=p_ori[2];
                p.pose.orientation.w=p_ori[3];
            #We call the service
            move_group_client(group_name, p, pos_tolerance, ang_tolerance)
            #print("Trajectory 1:",pos_x1[t],pos_y1[t],pos_z1[t])
            #print("tray 2 es:",pos_x2[t],pos_y2[t],pos_z2[t])
            #print("new trayectoria es",p)
            pos_xf.append(p.pose.position.x)
            pos_yf.append(p.pose.position.y)
            pos_zf.append(p.pose.position.z)
            ori_xf.append(p.pose.orientation.x)
            ori_yf.append(p.pose.orientation.y)
            ori_zf.append(p.pose.orientation.z)
            ori_wf.append(p.pose.orientation.w)
            count+=1


    '''
    #Error of the result trajectory
    tres_pos=[pos_xf[-1],pos_yf[-1],pos_zf[-1]]
    tres_ori=[ori_xf[-1],ori_yf[-1],ori_zf[-1],ori_wf[-1]]
    p.pose.position.x=tres_pos[0]
    p.pose.position.y=tres_pos[1]
    p.pose.position.z=tres_pos[2]
    p.pose.orientation.x=tres_ori[0]
    p.pose.orientation.y=tres_ori[1]
    p.pose.orientation.z=tres_ori[2]
    p.pose.orientation.w=tres_ori[3]
    tf_pos=[tf[0],tf[1],tf[2]]
    tf_ori=[tf[3],tf[4],tf[5],tf[6]]
    move_group_client(group_name, p, pos_tolerance, ang_tolerance)
    print("Final trajectory is")
    print(tres_pos, tres_ori)
    pos_sum=map(sub,tf_pos,tres_pos)
    dis_pos=math.sqrt(sum([x**2 for x in pos_sum])) #Euclidian distance
    #print(dis)
    dis_ori=abs(np.dot(tf_ori,tres_ori))
    dis=dis_pos+dis_ori*0.1
    print("El error es:")
    print(dis)
    '''

    #Uncomment for plots results.
    time=np.arange(0,3+0.1,0.1)#t_inc
    plt.figure()
    plt.plot(time,pos_x[b_s[0]],time,pos_x[b_s[1]],time,pos_xf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('X trajectory spline interpolation')
    plt.plot(time,pos_z[0])

    plt.figure()
    plt.plot(time,pos_y[b_s[0]],time,pos_y[b_s[1]],time,pos_yf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('Y trajectory spline interpolation')
    plt.plot(time,pos_z[0])

    plt.figure()
    plt.plot(time,pos_z[b_s[0]],time,pos_z[b_s[1]],time,pos_zf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('Z trajectory spline interpolation')
    plt.plot(time,pos_z[0])
    plt.show()
 
    return trajectory_gen_srvResponse(True)

rospy.init_node('trajectory_gen_node')
s = rospy.Service('trajectory_gen', trajectory_gen_srv, trajectory_gen)
print "Trajectory gen READY"
rospy.spin()

