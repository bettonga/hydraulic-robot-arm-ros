#!/usr/bin/env python3

import serial # pyserial
import os
import threading
import math
import time
import numpy as np

# Ros imports
import rospy
from arm_control.msg import ArmTip2D
from arm_control.srv import IsInRange, IsInRangeResponse
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

# Upper arm length
A 	= 250
# Forearm length
B 	= 250

# Starting position arm straight front
posX = 500
posY = 0
vel 	= 0
accel 	= 0
ShoulderAng, ElbowAng, WristAng = 0,0,0

ShoulderAngGoal, ElbowAngGoal, WristAngGoal = ShoulderAng, ElbowAng, WristAng
WristAngGlobal = ShoulderAng +ElbowAng +WristAng
WristAngGlobalGoal = 0

# Boundaries
shoulderMin = -45
shoulderMax = 15
elbowMin = -90
elbowMax = 0
wristMin = -97
wristMax = 110
wristGlobalMin = shoulderMin +elbowMin +wristMin
wristGlobalMax = shoulderMax +elbowMax +wristMax

def brutalExit():
	os.kill(os.getpid(), 9)

# Out of bounds check
def obs(x,y):
	# Check needed to make xyToJointAngles() happy.
	if math.sqrt((x)**2 + (y)**2) <= 500:

		ShoulderAng_check	= int(xyToJointAngles(x, y)[1])
		ElbowAng_check		= int(xyToJointAngles(x, y)[0])

		if not (ElbowAng_check > elbowMax or ElbowAng_check < elbowMin or ShoulderAng_check > shoulderMax or ShoulderAng_check < shoulderMin):
			return False
		else:
			return True
	else:
		return True

def jointAnglesToXY(shoulder, elbow):
	shoulder = math.radians(shoulder + 90)

	elbowX = + A*math.sin(shoulder)
	elbowY = - B*math.cos(shoulder)

	elbow = math.radians(elbow) + shoulder
	tipX = elbowX + A*math.sin(elbow)
	tipY = elbowY - B*math.cos(elbow)

	return [tipX, tipY]

def anglesCallback(data):
	# Calculate tip position from motor angle data
	global posX
	global posY
	global ShoulderAng, ElbowAng, WristAng

	ShoulderAng = data.data[0]
	ElbowAng = data.data[1]
	WristAng = data.data[2]

	posX = int(jointAnglesToXY(ShoulderAng, ElbowAng)[0])
	posY = int(jointAnglesToXY(ShoulderAng, ElbowAng)[1])
	WristAngGlobal = WristAng +ShoulderAng +ElbowAng

def lissajou(ang=np.pi/4, length=100, width=40, origX=300, origY=-300, n=100):
    T = np.linspace(0,2*np.pi,n)
    L = np.vstack((np.sin(T) , np.sin(2*T)))

    M = np.array([[length , 0],
                  [0 , width]])
    R = np.array([[np.cos(ang) , -np.sin(ang)],
                  [np.sin(ang) , np.cos(ang) ]])
    O = np.ones((2,n)) *np.array([[origX] ,[origY]])

    return R@M@L +O

def circle(radius, origX, origY, θi, n):
    T = np.linspace(θi,θi+2*np.pi,n)
    L = np.vstack((np.cos(T) , np.sin(T)))

    M = np.array([[radius , 0],
                  [0 , radius]])
    O = np.ones((2,n)) *np.array([[origX] ,[origY]])

    return M@L +O

def line(origX, origY, destX, destY, dl):
	n = int(np.round(np.sqrt((origX-destX)**2+(origY-destY)**2)/dl))
	T = np.linspace(0,1,n)
	X = origX + T*(destX-origX)
	Y = origY + T*(destY-origY)
	return np.vstack((X,Y))

def lines(dl=10):
	x1,y1 = 420, -200
	x2,y2 = 320, y1
	x3,y3 = x2, -370
	x4,y4 = 120, y3
	A = np.hstack((line(x1,y1,x2,y2, dl),
			   	   line(x2,y2,x3,y3, dl),
				   line(x3,y3,x4,y4, dl)))
	A = np.hstack((A,np.flip(A,1)))
	return A

def main():
	dl = 50
	waypt = lines(dl=dl)
	len=np.shape(waypt)[1]
	ε = 20
	i=0
	while not rospy.is_shutdown():
		tipxy.x = int(waypt[0,i])
		tipxy.y = int(waypt[1,i])
		tipxy.R = int(0)
		tip_xy_goal.publish(tipxy)
		rospy.loginfo("i: %d | dist: %f", i, math.dist([posX,posY],waypt[:,i]))
		if math.dist([posX,posY],waypt[:,i])<ε:
			i = (i+1)%len
		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('traj_node')#, anonymous=True)
		rospy.Subscriber("joint_angles_position", Int16MultiArray, anglesCallback)

		# Command to motor node
		tip_xy_goal = rospy.Publisher('tip_xy_goal', ArmTip2D, queue_size=10)
		tipxy = ArmTip2D()	# [x, y, r (, speed, accel)]

		rate = rospy.Rate(100) # 10hz

		main()

	except	KeyboardInterrupt:
		#brutalExit()
		quit()

	except rospy.ROSInterruptException:
		pass
