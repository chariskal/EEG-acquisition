#!/usr/bin/env python
#
# Parameters to check:
#  Subscriber: /pa10/emg_pos, /pa10/eeg_pos or /pa10/fusion_pos 
#  Input x,y,z in cm
#  If input from human hand --> add 0.6m to z coordinate (robot arm longer)
#  Control(line 119): Now 4 DoFs redundant with qref, weights k1, k2
# 
import roslib
import rospy
import time
import numpy as np
from numpy.linalg import *
from math import *
from pa10_remote.msg import PPCArmData
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist, Quaternion, Vector3, Vector3Stamped
from sensor_msgs.msg import ChannelFloat32
from math_utils import *
from pa10_kinematics import *

q = np.zeros([7],dtype=float) # q in rads
#pd = np.zeros([3],dtype=float)
pd = np.array([0.13,  0.15, 0.95]) # pd in meters

def updateArmStates(msg):

    global q
   
    for i in range (0,7):
      q[i] = msg.data[i]*pi/180.0 # pa10 in degrees, script in rads
      
def updateReference(msg):
    global pd 
    pd[0] = 1.0*msg.vector.x*0.01 #+0.2# (cm to meters)
    pd[1] = 1.0*msg.vector.y*0.01 
    pd[2] = 1.0*msg.vector.z*0.01 +0.6#-0.4# or +0.6 # robot arm longer 


if __name__ == '__main__':
    try:
        rospy.init_node('PA10_EE_Tracking_node')
        #Loop Rate
        rate_it = rospy.Rate(400)        
        # Create publishers
        pub_vel = rospy.Publisher('/pa10/qdot_cmd', Float64MultiArray, queue_size=1)
        pub_pcmd = rospy.Publisher('/pa10/pcmd', Float64MultiArray, queue_size=1)
        pub_prob = rospy.Publisher('/pa10/prob', Float64MultiArray, queue_size=1)
	      #Subscribers
        rospy.Subscriber("/pa10/joint_positions", Float64MultiArray, updateArmStates) #Arm Joints Position
        # /pa10/emg_pos, /pa10/eeg_pos, /pa10/fusion_pos
        rospy.Subscriber("/pa10/emg_pos", Vector3Stamped, updateReference) #Arm Joints Position

        qv = np.zeros ((7,1))

        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
        
            #Update Global Time
            t = rospy.Time.now().to_sec() - t0
            
            # Forward Kinematics --> Current Position & Position 
            p, R = pa10FKDHModified (q)

            print "EE Position:", p
            print "EE Orientation:", R
            print "q:", q*180.0/pi
            #Current Orientation (Vector Form)
            nt = np.array([R[0,0], R[1,0], R[2,0]])
            ot = np.array([R[0,1], R[1,1], R[2,1]])
            at = np.array([R[0,2], R[1,2], R[2,2]])

            
            #Desired Position
            #poeB = np.array([0.0,  0.641, 0.690])
            poeB = np.array([0.13,  0.15, 0.95])
            poeB[0] = pd[0]
            poeB[1] = pd[1]
            poeB[2] = pd[2] 
            #poeB = np.array([0.22,  0.28, 0.8])
            
        
            #Desired Orientation
            """RoeB = np.array([[  0.0,  -1.0,   0.0],
                             [  0.0,   0.0,   1.0],
                             [ -1.0,   0.0,   0.0]])"""
                             
            RoeB = np.array([[  1.0,   0.0,   0.0],
                             [  0.0,   1.0,   0.0],
                             [  0.0,   0.0,   1.0]])
                         
            
            #Desired Orientation Vector Form (Fixed)
            nd = RoeB[:,0]
            od = RoeB[:,1]
            ad = RoeB[:,2]
            
            #Error --> End Effector Frame
            ed = np.array([0.0, 0.0, 0.0])
            
            #Pose Trajectory Tracking
            #Position
            ed[0] = p[0] - poeB[0] 
            ed[1] = p[1] - poeB[1]
            ed[2] = p[2] - poeB[2]
            
            qref = np.array([70, 40, -5, -60, -15, 0, -80])*pi/180.0
            
            #Orientation (Fixed)
            #ed[3:6] = -0.5*(np.cross(nt,nd) + np.cross(ot,od) + np.cross(at,ad))
           
            J = pa10Jacobian(q)
            J34 = J[0:3,0:4]   
            #print "Task Space Errors:", ed
            
            k1 = 1.2 #np.array([0.1, 0.4, 0.1, 0.4, 0.1, 0.4, 0.1])
            k2 = 0.6 #np.array([0.1, 0.4, 0.1, 0.4, 0.1, 0.4, 0.1])
            qv34 = -k1 * np.dot(pinv(J34), ed) -k2 * np.dot( (np.eye(4) - np.dot(pinv(J34), J34)), q[0:4]-qref[0:4] )
            qv = np.concatenate([qv34,np.array([0,0,0])])
            print "qdot:", qv
            
            #Fill CMD Msg             
            cmd = Float64MultiArray()
            cmd.layout.dim = [MultiArrayDimension()]
            cmd.layout.dim[0].size = 7
            cmd.layout.dim[0].stride = 7
            cmd.layout.data_offset = 0
            cmd.data = list (float (qvelt) for qvelt in qv) 
            
            #Fill pcmd Msg             
            pcmd = Float64MultiArray()
            pcmd.layout.dim = [MultiArrayDimension()]
            pcmd.layout.dim[0].size = 3
            pcmd.layout.dim[0].stride = 3
            pcmd.layout.data_offset = 0
            pcmd.data = list (float (perror) for perror in poeB) # ed
            
            #Fill prob Msg             
            prob = Float64MultiArray()
            prob.layout.dim = [MultiArrayDimension()]
            prob.layout.dim[0].size = 3
            prob.layout.dim[0].stride = 3
            prob.layout.data_offset = 0
            prob.data = list (float (perror) for perror in p) # ed
            
            print "ed:", ed
            print "cmd_data", cmd.data
            
            pub_vel.publish(cmd)
            pub_pcmd.publish(pcmd)
            pub_prob.publish(prob)
            
            rate_it.sleep()
        rospy.spin()

    except rospy.ROSInterruptException: pass
