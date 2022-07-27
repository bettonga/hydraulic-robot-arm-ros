#!/usr/bin/env python3

import serial # pyserial
import os
import threading
import math
import time

# Ros imports
import rospy
from arm_control.msg import ArmTip2D
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


# Upper arm length
A 	= 250
# Forearm length
B 	= 250

# Starting position arm straight front
posX 	= 500
posY 	= 0
vel 	= 0
accel 	= 0
ShoulderAng, ElbowAng, WristAng = 0,0,0

posXGoal, posYGoal = posX, posY
ShoulderAngGoal, ElbowAngGoal, WristAngGoal = ShoulderAng, ElbowAng, WristAng
WristAngGlobal = ShoulderAng +ElbowAng +WristAng
WristAngGlobalGoal = 0

move_posXGoal, move_posYGoal = posX, posY
move_WristAngGlobalGoal = WristAngGlobal

# Boundaries
shoulderMin = -45
shoulderMax = 15
elbowMin = -90
elbowMax = 0
wristMin = -97
wristMax = 110
wristGlobalMin = shoulderMin +elbowMin +wristMin
wristGlobalMax = shoulderMax +elbowMax +wristMax

# Init joystick axes
LY = LX = RX = 0
buttons = [0,0,0,0,0,0,0,0,0,0,0]


"""
			  B        ._
	elbow  .__________________/­
	       /                  \._  effector (x,y)
	      /
	     /
	    /  A
	   /
	  /
	./
    shoulder

"""


def brutalExit():
	os.kill(os.getpid(), 9)



# IK ----------------------- start

def xyToJointAngles(x,y):
	# a quich hack to make the math happy
	x -= 0.000000001
	y += 0.000000001

	# IK calculation
	ang1 = math.acos( ( (x**2)+(y**2)-(A**2)-(B**2) ) / (2*A*B) )				# elbow joint
	ang2 = math.atan(y/x) + math.asin( (B*math.sin(ang1) ) / (math.sqrt((x**2)+(y**2)))) 	# shoulder joint

	if x < 0:
		ang2 -= math.pi

	return (-math.degrees(ang1), math.degrees(ang2))


def wristGlobalToJointAngle(wrist, shoulder, elbow):
	jointAngle = int(wrist) - int(shoulder) - int(elbow)
	jointAngle = min(wristMax, max(wristMin, jointAngle))	# make sure -90<wristAngle<90
	return jointAngle

# IK ----------------------- end



# FK ----------------------- start

def jointAnglesToXY(shoulder, elbow):
	shoulder = math.radians(shoulder + 90)

	elbowX = + A*math.sin(shoulder)
	elbowY = - B*math.cos(shoulder)

	elbow = math.radians(elbow) + shoulder
	tipX = elbowX + A*math.sin(elbow)
	tipY = elbowY - B*math.cos(elbow)

	return [tipX, tipY]

# FK ----------------------- end



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



# ROS callbacks ------------ start

def joyCallback(data):
	global vel
	global accel
	global LY, LX, RX
	global buttons

	axes = data.axes

	# In case the joystick has a wrong scalin, speed is limited to 1 (not tested)
	for i in range(len(axes)):
		if math.fabs(axes[i]) > 1:
			axes[i] = axes[i] / math.fabs(axes[i])

	LY = axes[1]
	LX = axes[0]
	RX = axes[2]
	vel = int(max(abs(LX),abs(LY))*15)
	buttons = data.buttons


def moveCallback(data):
	global move_posXGoal
	global move_posYGoal
	global move_WristAngGlobalGoal
	global move_vel
	global move_accel

	if not obs(data.x, data.y):
		move_posXGoal 		= data.x
		move_posYGoal 		= data.y
		move_WristAngGlobalGoal	= data.R
		# move_vel 		= data.speed
		# move_accel 		= data.accel

		# print(data)
		# update()


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

# ROS callbacks ------------ end


def update():
	global ShoulderAngGoal, ElbowAngGoal, WristAngGoal
	global posXGoal, posYGoal

	angleCommand.data = [int(ShoulderAngGoal),
						 int(ElbowAngGoal),
						 int(WristAngGoal),
						 vel,
						 accel]
	motor_angles_goal.publish(angleCommand)

	tipxy.x = int(posX)
	tipxy.y = int(posY)
	tipxy.R = int(WristAngGlobal)

	tip_xy_position.publish(tipxy)




def main():
	old = [0,0,0]
	vel_wrist = 5
	i_joy = 0

	global WristAngGlobalGoal
	global ShoulderAngGoal, ElbowAngGoal, WristAngGoal
	global posXGoal, posYGoal

	while not rospy.is_shutdown():

		# Use traj controller if button 1 is pressed (A)
		if buttons[1]:
			ShoulderAngGoal = int(xyToJointAngles(move_posXGoal, move_posYGoal)[1])
			ElbowAngGoal = int(xyToJointAngles(move_posXGoal, move_posYGoal)[0])
			WristAngGlobalGoal = int(move_WristAngGlobalGoal)

		elif i_joy>=10:	# so that joystick command rate is 10hz
			i_joy=0
			# Update position if joystick active
			if max(abs(LX),abs(LY),abs(RX)) != 0:

				# use IK joystick control if button 5 is pressed (RB)
				if buttons[5]:

					# Update if in range
					if not obs(posXGoal + LX * vel, posYGoal + LY * vel):
						posXGoal += LX * vel
						posYGoal += LY * vel
						posXGoal = int(posXGoal)
						posYGoal = int(posYGoal)
						ShoulderAngGoal = int(xyToJointAngles(posXGoal, posYGoal)[1])
						ElbowAngGoal = int(xyToJointAngles(posXGoal, posYGoal)[0])

				# Use FK joystick control as default
				else:
					ShoulderAngGoal	+= LY * vel * 0.11
					ElbowAngGoal	+= LX * vel * 0.2
					ElbowAngGoal = max(elbowMin, min(elbowMax, int(ElbowAngGoal)))
					ShoulderAngGoal = max(shoulderMin, min(shoulderMax, int(ShoulderAngGoal)))
					posXGoal = int(jointAnglesToXY(ShoulderAngGoal, ElbowAngGoal)[0])
					posYGoal = int(jointAnglesToXY(ShoulderAngGoal, ElbowAngGoal)[1])

				# In both cases, update wrist angle
				WristAngGlobalGoal += RX * vel_wrist
		i_joy += 1

		WristAngGlobalGoal = min(wristGlobalMax, max(wristGlobalMin, int(WristAngGlobalGoal)))
		WristAngGoal = WristAngGlobalGoal -ShoulderAngGoal -ElbowAngGoal
		WristAngGoal = min(wristMax, max(wristMin, int(WristAngGoal)))

		# # Send command if position has changed
		# new = [ShoulderAngGoal, ElbowAngGoal, WristAngGoal]
		# #print(abs(new[2] - old[2]))
		# if old != new: #and not (abs(new[2] - old[2]) == 1):
		update()
			# old = new

		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('IK_node')#, anonymous=True)
		rospy.Subscriber("/joy", Joy, joyCallback)
		rospy.Subscriber("tip_xy_goal", ArmTip2D, moveCallback)
		rospy.Subscriber("joint_angles_position", Int16MultiArray, anglesCallback)

		# Command to motor node
		motor_angles_goal = rospy.Publisher('joint_angles_goal', Int16MultiArray, queue_size=10)
		angleCommand = Int16MultiArray()	# [shoulder, elbow, wrist (, speed, accel)]

		# Report position to control
		tip_xy_position = rospy.Publisher('tip_xy_position', ArmTip2D, queue_size=10)
		tipxy = ArmTip2D()

		ros_freq = 100
		rate = rospy.Rate(ros_freq) # 10hz

		main()

	except	KeyboardInterrupt:
		#brutalExit()
		quit()

	except rospy.ROSInterruptException:
		pass
