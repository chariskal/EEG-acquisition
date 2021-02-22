#!/usr/bin/env python

import roslib
import rospy
import time
import numpy as np
from numpy.linalg import *
from math import *
from pa10_remote.msg import PPCArmData
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import ChannelFloat32
from math_utils import *
from pa10_kinematics import *

q = np.zeros([7],dtype=float)
loadcells = np.zeros([5],dtype=float)
med_force = 0.0
rho_pose = np.array([0.0, 0.0, 0.0, 0.0])

def updateLoadCells (msg):
        global loadcells
        global med_force   
 
        loadcells[0] = msg.values[0]
        loadcells[1] = msg.values[1]
        loadcells[2] = msg.values[2]
        loadcells[3] = msg.values[3]
        med_force = msg.values[4]


def updateArmStates(msg):

    global q
   
    for i in range (0,7):
      q[i] = msg.data[i]*pi/180.0


def dT(xi,rho):
    return 1.0/(rho*(1.0-xi*xi))
        
def performanceFunction(rho_0, rho_f, lamda, t): 
    return (rho_0 - rho_f)*exp(-lamda*t) + (rho_f)
        
def getTransformed(rho,e,M,name):
    xi = ( e - (M[1]*rho[1]+M[0]*rho[0])/2.0)/((M[1]*rho[1] - M[0]*rho[0])/2.0 )
    if xi > 0.99:
       xi = 0.99
       rospy.logwarn("PPC in "+name+" violated")
    if xi < -0.99:
       xi = -0.99
       rospy.logwarn("PPC in "+name+" violated")       
    return xi, atanh(xi)
        
def ppcForceKinematics(Jinv,ex, force, ez, eo, Li, t):
    
    global rho_pose
    
    rho_0 = 5.0
        
    rhof_fi = 0.25
    kfi = 0.5
    lfi = 0.5
    Ml = 1.0
        
    rhof_x = 0.3
    rhof_z = 0.3
        
    lx = 0.5
    lz = 0.5
    kx = 0.5
    kz = 0.5
        
    rhof_o = 1.0
    ko = 1.1
    lo = 0.5

    #force --> along Y-AXIS  
    rho_fiU = performanceFunction(rho_0, rhof_fi,lfi,t)
    rho_fiL = performanceFunction(rho_0, rhof_fi/Ml,lfi,t)
        
    fd = 0.0
    xi_fi,eT_fi = getTransformed([-rho_fiU, rho_fiU],force,[1.0, 1.0],"force")
    #xi_fi,eT_fi = getTransformed([-rho_fiU, rho_fiU],-(force-fd),[1.0, 1.0],"force")
    dT_fi = dT(xi_fi,rho_fiU)
        
    u_fi = -kfi*(1.0/dT_fi+dT_fi)*eT_fi
    u_fi = saturation(u_fi,-0.1,0.1)
        
        
    """print "fd:",fd
    print "f:",force
    print "xf:",xi_fi
    print "eT_fi:",eT_fi
    print " dT_fi:", dT_fi
    print " rho_fiU:", rho_fiU"""
        
    #x
    rho_x = performanceFunction(rho_0,rhof_x,lx,t)
        
        
    xi_x,eT_x = getTransformed([-rho_x, rho_x],ex,[1.0, 1.0],"x-axis")
    dT_x = dT(xi_x,rho_x)
        
    u_x = -kx*(1.0/dT_x+dT_x)*eT_x
        
    #z
    rho_z = performanceFunction(rho_0,rhof_z,lz,t)
        
        
    xi_z,eT_z = getTransformed([-rho_z, rho_z],ez,[1.0, 1.0],"z-axis")
    dT_z = dT(xi_z,rho_z)
        
    u_z = -kz*(1.0/dT_z+dT_z)*eT_z
    u_z = saturation(u_z,-0.5,0.5)        
        
    #o
    rho_o = performanceFunction(rho_0, rhof_o,lo,t)
        
    rho_pose = np.array([rho_x, rho_fiU, rho_z, rho_o])
        
        
    xi_o=np.array([0.0,0.0,0.0])
    eT_o=np.array([0.0,0.0,0.0])
    dT_o=np.array([0.0,0.0,0.0])
        
    for x in range(0,3):
        xi_o[x],eT_o[x] = getTransformed([-rho_o, rho_o],eo[x],[1.0, 1.0],"orient")
        dT_o[x] = dT(xi_o[x],rho_o)
        
        
    dT_oM = np.diag(1.0/dT_o+dT_o)
        
    u_o = -ko*np.dot(np.dot(Li,dT_oM),eT_o)
        
    for x in range(0,3):
        u_o[x] = saturation(u_o[x],-0.15,0.15) #~9deg
        
    udes = np.array([u_x, u_fi, u_z, u_o[0],u_o[1],u_o[2]]).reshape(6,)
        
    #print "udes:",udes
        
    return np.dot(Jinv,udes)

    

if __name__ == '__main__':
    try:
        rospy.init_node('PA10ForceControl')
        #Loop Rate
        rate_it = rospy.Rate(10)        
        # Create publishers
        pub_vel = rospy.Publisher('/pa10/qdot_cmd', Float64MultiArray, queue_size=1)
        #Data Publisher
        data_pub = rospy.Publisher("/ppc_data", PPCArmData, queue_size=1)
	#Subscribers
        rospy.Subscriber("/pa10/joint_positions", Float64MultiArray, updateArmStates) #Arm Joints Position
        rospy.Subscriber("/load_cells/data", ChannelFloat32, updateLoadCells)

        qv = np.zeros ((7,1))

        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
        
            #Update Global Time
            t = rospy.Time.now().to_sec() - t0
            
            # Forward Kinematics --> Current Position & Position 
            p, R = pa10FKDHModified (q)

            #print p
            #print R

            #Current Orientation (Vector Form)
            nt = np.array([R[0,0], R[1,0], R[2,0]])
            ot = np.array([R[0,1], R[1,1], R[2,1]])
            at = np.array([R[0,2], R[1,2], R[2,2]])

            
            #Desired Position
            poeB = np.array([0.0,  0.641, 0.690])
        
            #Desired Orientation
            RoeB = np.array([[  0.0,  -1.0,   0.0],
                             [  0.0,   0.0,   1.0],
                             [ -1.0,   0.0,   0.0]])
            
            #Desired Orientation Vector Form (Fixed)
            nd = RoeB[:,0]
            od = RoeB[:,1]
            ad = RoeB[:,2]
            
            #Error --> End Effector Frame
            ed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            #Pose Trajectory Tracking
            #Position
            ed[0] = p[0] - (poeB[0] + 0.05*cos(2.0*pi*0.05*t)) 
            ed[1] = p[1] - (poeB[1] + 0.0*sin(2.0*pi*0.05*t))
            ed[2] = p[2] - (poeB[2] + 0.05*sin(2.0*pi*0.05*t)) 
            
            
            #Orientation (Fixed)
            ed[3:6] = -0.5*(np.cross(nt,nd) + np.cross(ot,od) + np.cross(at,ad))
        

            L = -0.5*(np.dot(sskew(nt),sskew(nd))+np.dot(sskew(ot),sskew(od))+np.dot(sskew(at),sskew(ad)))
           
            J = pa10Jacobian(q)
               
            #print "Task Space Errors:", ed
            #qv = ppcForceKinematics(pinv(J),ed[0], med_force, ed[2], ed[3:6], inv(L), t)
            qv = ppcForceKinematics(pinv(J), ed[0], ed[1], ed[2], ed[3:6], inv(L), t)
          
            #Fill CMD Msg             
            cmd = Float64MultiArray()
            cmd.layout.dim = [MultiArrayDimension()]
            cmd.layout.dim[0].size = 7
            cmd.layout.dim[0].stride = 7
            cmd.layout.data_offset = 0
            cmd.data = list (float (qvelt) for qvelt in qv) 
            
            #print cmd.data
            pub_vel.publish (cmd)

            #Fill Complete Data Msg
            data = PPCArmData()
            data.t = t
            data.q = list(q)
            
            data.qddot = list(qv)
            
               
            data.p[0] = p[0]
            data.p[1] = p[1]
            data.p[2] = p[2]
               
            data.R[0] = R[0,0]
            data.R[1] = R[0,1]
            data.R[2] = R[0,2]
            data.R[3] = R[1,0]
            data.R[4] = R[1,1]
            data.R[5] = R[1,2]
            data.R[6] = R[2,0]
            data.R[7] = R[2,1]
            data.R[8] = R[2,2]
               
            data.force = med_force
            data.rho_pose_f = list(rho_pose)
               
            data_pub.publish(data)
            
            rate_it.sleep()
        rospy.spin()

    except rospy.ROSInterruptException: pass
