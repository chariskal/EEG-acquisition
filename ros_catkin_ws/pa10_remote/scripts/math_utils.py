#!/usr/bin/env python

import roslib
import rospy
from numpy import zeros, array, asarray
from numpy import sum as npsum

from math import sqrt, atan2, sin, pi, asin, cos, acos
from geometry_msgs.msg import Vector3, Quaternion


#Saturation
def saturation(u,umin,umax):
	if u>umax:
		u=umax
	elif u<umin:
		u=umin
	
	return u


#Quaternion to Rotation Matrix - (Reb)
def quat2rotmtx (quat):
	rotmtx=zeros((3,3))
  	rotmtx[0][0] = quat.w*quat.w + quat.x*quat.x - quat.y*quat.y - quat.z*quat.z
  	rotmtx[0][1] = 2.0*(quat.x*quat.y - quat.w*quat.z)
  	rotmtx[0][2] = 2.0*(quat.x*quat.z + quat.w*quat.y)
  	rotmtx[1][0] = 2.0*(quat.x*quat.y + quat.w*quat.z)
  	rotmtx[1][1] = quat.w*quat.w - quat.x*quat.x + quat.y*quat.y - quat.z*quat.z
  	rotmtx[1][2] = 2.0*(quat.y*quat.z - quat.w*quat.x)
  	rotmtx[2][0] = 2.0*(quat.x*quat.z - quat.w*quat.y)
  	rotmtx[2][1] = 2.0*(quat.y*quat.z + quat.w*quat.x)
  	rotmtx[2][2] = quat.w*quat.w - quat.x*quat.x - quat.y*quat.y + quat.z*quat.z

	return rotmtx

#Rotation Matrix to Quaternion - (Reb)
def rotmtx2quat (rotmtx):
	q = Quaternion()
	q.w = 1.0

	tr = rotmtx[0][0] + rotmtx[1][1] + rotmtx[2][2]

	if tr>0:
		sqtrp1 = sqrt(tr + 1.0);
		sqtrp1x2 = 2.0*sqtrp1;

		q.w = 0.5*sqtrp1;
		q.x = (rotmtx[2][1] - rotmtx[1][2])/sqtrp1x2
		q.y = (rotmtx[0][2] - rotmtx[2][0])/sqtrp1x2
		q.z = (rotmtx[1][0] - rotmtx[0][1])/sqtrp1x2

		return q
	else:
		d0=rotmtx[0][0]
		d1=rotmtx[1][1]
		d2=rotmtx[2][2]

		if ((d1 > d0) and (d1 > d2)):
			sqdip1 = sqrt(d1 - d0 - d2 + 1.0 )
			q.y = 0.5*sqdip1

			if abs(sqdip1)>1e-7:
				sqdip1 = 0.5/sqdip1

			q.w = (rotmtx[0][2] - rotmtx[2][0])*sqdip1
			q.x = (rotmtx[1][0] + rotmtx[0][1])*sqdip1
			q.z = (rotmtx[2][1] + rotmtx[1][2])*sqdip1

			return q

		elif (d2 > d0):
			#max value at R(3,3)
			sqdip1 = sqrt(d2 - d0 - d1 + 1.0 )

			q.z = 0.5*sqdip1;

			if abs(sqdip1)>1e-7:
				sqdip1 = 0.5/sqdip1

			q.w = (rotmtx[1][0] - rotmtx[0][1])*sqdip1
			q.x = (rotmtx[0][2] + rotmtx[2][0])*sqdip1
			q.y = (rotmtx[2][1] + rotmtx[1][2])*sqdip1

			return q

		else:
			# max value at R(1,1)
			sqdip1 = sqrt(d0 - d1 - d2 + 1.0 )

			q.x = 0.5*sqdip1

			if abs(sqdip1) > 1e-7:
				sqdip1 = 0.5/sqdip1

			q.w = (rotmtx[2][1] - rotmtx[1][2])*sqdip1
			q.y = (rotmtx[1][0] + rotmtx[0][1])*sqdip1
			q.z = (rotmtx[0][2] + rotmtx[2][0])*sqdip1

			return q

#Quaternion to Euler-ZYX
def quat2eulerZYX (q):
    euler = Vector3()
    tol = quat2eulerZYX.tolerance
    
    qww, qxx, qyy, qzz = q.w*q.w, q.x*q.x, q.y*q.y, q.z*q.z
    qwx, qxy, qyz, qxz= q.w*q.x, q.x*q.y, q.y*q.z, q.x*q.z
    qwy, qwz = q.w*q.y, q.w*q.z

    test = -2.0 * (qxz - qwy)
    if test > +tol:
        euler.x = atan2 (-2.0*(qyz-qwx), qww-qxx+qyy-qzz)
        euler.y = +0.5 * pi
        euler.z = 0.0

	return euler

    elif test < -tol:
        euler.x = atan2 (-2.0*(qyz-qwx), qww-qxx+qyy-qzz)
        euler.y = -0.5 * pi
        euler.z = tol

	return euler

    else:
        euler.x = atan2 (2.0*(qyz+qwx), qww-qxx-qyy+qzz)
        euler.y = asin (test)
        euler.z = atan2 (2.0*(qxy+qwz), qww+qxx-qyy-qzz)

	return euler
quat2eulerZYX.tolerance=0.99999

#Euler-ZYX to Quaternion
def eulerZYX2quat(euler):
	q = Quaternion()
	cpsi = cos (0.5 * euler.z)
	spsi = sin (0.5 * euler.z)
  	ctheta = cos (0.5 * euler.y)
	stheta = sin (0.5 * euler.y)
  	cphi = cos (0.5 * euler.x)
	sphi = sin (0.5 * euler.x)
  	q.w = cphi*ctheta*cpsi + sphi*stheta*spsi
  	q.x = sphi*ctheta*cpsi - cphi*stheta*spsi
  	q.y = cphi*stheta*cpsi + sphi*ctheta*spsi
  	q.z = cphi*ctheta*spsi - sphi*stheta*cpsi

	return quat_normalize(q)

#Euler-ZYX to Rotation Matrix - (Reb)
def eulerZYX2rotmtx(euler):
	rotmtx=zeros((3,3))
  	cpsi   = cos(euler.z)
	spsi   = sin(euler.z)
  	ctheta = cos(euler.y)
	stheta = sin(euler.y)
  	cphi   = cos(euler.x)
	sphi   = sin(euler.x)
  	#Calculate rotation matrix
  	rotmtx[0][0] = cpsi * ctheta
  	rotmtx[0][1] = sphi * cpsi * stheta - cphi * spsi
  	rotmtx[0][2] = cphi * cpsi * stheta + sphi * spsi
  	rotmtx[1][0] = spsi * ctheta
  	rotmtx[1][1] = sphi * spsi * stheta + cphi * cpsi
  	rotmtx[1][2] = cphi * spsi * stheta - sphi * cpsi
  	rotmtx[2][0] = -stheta
  	rotmtx[2][1] = sphi * ctheta
  	rotmtx[2][2] = cphi * ctheta

	return rotmtx

#Rotation Matrix to Euler-ZYX - (Reb)
def rotmtx2eulerZYX (mtx):

	tolerance = rotmtx2eulerZYX.tolerance
	test = -mtx [2][0]

	if test > +tolerance:
        	phi = atan2 (-mtx[1][2], mtx[1][1])
        	theta = +0.5 * pi
        	psi = 0.0
    	elif test < -tolerance:
        	phi = atan2 (-mtx[1][2], mtx[1][1])
        	theta = -0.5 * pi
        	psi = 0.0
    	else:
        	phi = atan2 (mtx[2][1], mtx[2][2])
        	theta = asin (-mtx[2][0])
        	psi = atan2 (mtx[1][0], mtx[0][0])

    	return Vector3(phi, theta, psi)
rotmtx2eulerZYX.tolerance = 0.99999


def quat2AngleAxis(qx,qy,qz,qw):
    n = sqrt (qw*qw + qx*qx + qy*qy + qx*qx) + 0.001
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    angle = 2.0 * acos(qw)
    s = sin(0.5*angle)
    #x = qx / sqrt(1-qw*qw)
    #y = qy / sqrt(1-qw*qw)
    #z = qz / sqrt(1-qw*qw)
    x = qx / s
    y = qy / s
    z = qz / s
    aaVector = array([angle,x,y,z])
    return aaVector

def sskew(a):
    B = array([[0.0, -a[2], a[1]],
               [a[2], 0.0, -a[0]],
               [-a[1], a[0],0.0]]).reshape(3,3)
    return B
