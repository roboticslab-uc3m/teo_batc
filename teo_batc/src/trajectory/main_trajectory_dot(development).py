#!/usr/bin/env python

import rospy
from teo_moveit.srv import *
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
        pos_tolerance= 0.5
        ang_tolerance= 0.5
        t_inc=0.1
        for t in np.arange(0,3,t_inc):
            p.pose.position.x = interpolate.splev(t,s_x[req.i[0]],der=0)
            p.pose.position.y = interpolate.splev(t,s_y[req.i[0]],der=0)
            p.pose.position.z = interpolate.splev(t,s_z[req.i[0]],der=0)
            p.pose.orientation.x = interpolate.splev(t,s_ox[req.i[0]],der=0)
            p.pose.orientation.y = interpolate.splev(t,s_oy[req.i[0]],der=0)
            p.pose.orientation.z = interpolate.splev(t,s_oz[req.i[0]],der=0)
            p.pose.orientation.w = interpolate.splev(t,s_ow[req.i[0]],der=0)
            #We call the service
            move_group_client(group_name, p, pos_tolerance, ang_tolerance)
            #print(p)
    else:
        print("Voy a generar la nueva trayectoria")
        #First we call the base solution generation
        #tf=[(pos_x2[-1]+pos_x1[-1])/2,(pos_y2[-1]+pos_y1[-1])/2,(pos_z2[-1]+pos_z1[-1])/2,(ori_x2[-1]+ori_x1[-1])/2,(ori_y2[-1]+ori_y1[-1]),(ori_z2[-1]+ori_z1[-1])/2,(ori_w2[-1]+ori_w1[-1])/2,0,0,0,0,0,0,0]
     #Define the list where we gonna save the trajectory
        #tf=[0.49938, -0.39086, 0.63233, 0.074214, -0.74326, -0.26863, 0.60819, 0,0,0,0,0,0,0]
	#tf_pos=[0.49938, -0.39086, 0.63233]
	#tf_ori=[0.074214, -0.74326, -0.26863, 0.60819]
	tf_pos=[0.28875, -0.30115, 0.93534]
	tf_ori=[-0.0014619, 0.93545, -0.013458, -0.35319]
	tf_pos=[-0.38565,-0.64816, 0.35461]
	tf_ori=[-0.61651, -0.18617, -0.46419, 0.60809]
	
        pos_xf=[]
        pos_yf=[]
        pos_zf=[]
        ori_xf=[]
        ori_yf=[]
        ori_zf=[]
        ori_wf=[]
        (b_s,w1,w2)=base_trajectory_gen(tf_pos, tf_ori)
        p=PoseStamped()
        p.header.frame_id = "base_link"
        group_name= "right_arm"
        pos_tolerance= 0.05
        ang_tolerance= 0.05
        count=0
        t_inc=0.1
        for t in np.arange(0,3+t_inc,t_inc):
            #Weghted sum
            p.pose.position.x= interpolate.splev(t,s_x[b_s[0]],der=0)*w1[0]+interpolate.splev(t,s_x[b_s[1]],der=0)*w2[0]
            p.pose.position.y= interpolate.splev(t,s_y[b_s[0]],der=0)*w1[1]+interpolate.splev(t,s_y[b_s[1]],der=0)*w2[1]
            p.pose.position.z= interpolate.splev(t,s_z[b_s[0]],der=0)*w1[2]+interpolate.splev(t,s_z[b_s[1]],der=0)*w2[2]
            #We use the funcion slerp to calculate the new quaternion 
           #print(p.pose.position.x)
           #print(p.pose.position.y)
           #print(p.pose.position.z)
            """
            #TODO Spline individual interpolation not work with 
            quaternions so
            i should find some other way to do this
            """

            '''
            We get the orientation trajectories for the instanti
            and trajectories b_s[0]
            and b_s[1].
            The first step to do is to interpolate the different points
            extracted from the trajectories '''
            #If we are in the last step the orienation is the same final state
            if t==3:
                p1=[s_ox[b_s[0]][-1],s_oy[b_s[0]][-1],s_oz[b_s[0]][-1],s_ow[b_s[0]][-1]]
                p2=[s_ox[b_s[1]][-1],s_oy[b_s[1]][-1],s_oz[b_s[1]][-1],s_ow[b_s[1]][-1]]
                print("orientaciones finals")
                print(p1)
                print(p2)
            else:
                #First we get what two points we need to interp. step will be the  
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
                #print("las trayectorias de las orientaciones son:")
                #print(p1)
                #print(p2)
                p_ori=interp(p1,p2,w1[3])
                #print(p_ori)

                p.pose.orientation.x=p_ori[0];
                p.pose.orientation.y=p_ori[1];
                p.pose.orientation.z=p_ori[2];
                p.pose.orientation.w=p_ori[3];
            #We call the servic
            '''
            p.pose.position.x= interpolate.splev(t,s_x[2],der=0)
            p.pose.position.y= interpolate.splev(t,s_y[2],der=0)
            p.pose.position.z= interpolate.splev(t,s_z[2],der=0)
            p10=[s_ox[b_s[0]][step],s_oy[b_s[0]][step],s_oz[b_s[0]][step],s_ow[b_s[0]][step]]
            p11=[s_ox[b_s[0]][step+1],s_oy[b_s[0]][step+1],s_oz[b_s[0]][step+1],s_ow[b_s[0]][step+1]]
            p1=interp(p10,p11,t-step) 
            p.pose.orientation.x=p1[0]
            p.pose.orientation.y=p1[1]
            p.pose.orientation.z=p1[2]
            p.pose.orientation.w=p1[3]
            '''

            #move_group_client(group_name, p, pos_tolerance, ang_tolerance)
            #print("trayectoria 1 es:",pos_x1[t],pos_y1[t],pos_z1[t])
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
    #Plot section (we plot the base trajectories and the result one)
    time=np.arange(0,3+0.1,0.1)#t_inc
    #print(time)
    #print(pos_x[2])
    ###THIS PART IS USED FOR GETTING THE ERROR EXPERIMENT
    #Error between the got state and the goal one.
    tres_pos=[pos_xf[-1],pos_yf[-1],pos_zf[-1]]
    tres_ori=[p_ori[0],p_ori[1],p_ori[2],p_ori[3]]
    p.pose.position.x=tres_pos[0]
    p.pose.position.y=tres_pos[1]
    p.pose.position.z=tres_pos[2]
    p.pose.orientation.x=tres_ori[0]
    p.pose.orientation.y=tres_ori[1]
    p.pose.orientation.z=tres_ori[2]
    p.pose.orientation.w=tres_ori[3]
    #move_group_client(group_name, p, pos_tolerance, ang_tolerance)
    print("trayectoria final es")
    print(tres_pos, tres_ori)
    pos_sum=map(sub,tf_pos,tres_pos)
    dis_pos=math.sqrt(sum([x**2 for x in pos_sum])) #Euclidian distance
    #print(dis)
    dis_ori=2*(1-np.dot(tf_ori,tres_ori))
    dis=dis_pos+dis_ori
    print("El error es:")
    print(dis)
    ''' 
    plt.figure()
    plt.plot(time,pos_x[3])
    plt.axis([0, 3, -1, 1])
    plt.title('X trajectory spline interpolation')

    plt.figure()
    plt.plot(time,pos_y[3])
    plt.axis([0, 3, -1, 1])
    plt.title('Y trajectory spline interpolation')

    plt.figure()
    plt.plot(time,pos_z[3])
    plt.axis([0, 3, -1, 1])
    #plt.axis([0, 3, -1.5, 1.5])
    plt.title('Z trajectory spline interpolation')
    plt.show()

    #d_time=np.arange(0,4,1)
    print(pos_x[b_s[0]])
    print(pos_xf)
    plt.figure()
    plt.plot(time,pos_x[b_s[0]], time, pos_x[b_s[1]], time,pos_xf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('X trajectory spline interpolation')

    plt.figure()
    plt.plot(time,pos_y[b_s[0]], time, pos_y[b_s[1]], time,pos_yf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('Y trajectory spline interpolation')

    plt.figure()
    plt.plot(time,pos_z[b_s[0]], time, pos_z[b_s[1]], time,pos_zf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('Z trajectory spline interpolation')
    plt.show()

    '''
    '''
    plt.figure()
    plt.plot(time, ori_x[b_s[0]], time, ori_x[b_s[1]], time,ori_xf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('ORIX trajectory spline interpolation')

    plt.figure()
    plt.plot(time, ori_y[b_s[0]], time, ori_y[b_s[1]], time,ori_yf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('oriy trajectory spline interpolation')

    plt.figure()
    plt.plot(time, ori_z[b_s[0]], time, ori_z[b_s[1]], time,ori_zf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('oriz trajectory spline interpolation')

    plt.plot(time,ori_w[b_s[0]], time, ori_w[b_s[1]] , time,ori_wf)
    plt.axis([0, 3, -1.5, 1.5])
    plt.title('oriw trajectory spline interpolation')
    plt.show()
    '''
    return trajectory_gen_srvResponse(True)

rospy.init_node('trajectory_gen_node')
s = rospy.Service('trajectory_gen', trajectory_gen_srv, trajectory_gen)
print "Trajectory gen READY"
rospy.spin()

