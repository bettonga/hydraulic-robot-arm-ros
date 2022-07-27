#!/usr/bin/env python3

import serial # pyserial
import tkinter as tk
import time
import os
import threading
import IK_node

# a bit ugly math import...
from math import cos, sin, tan, pi, sqrt
import math

# Ros imports
import rospy
from arm_control.msg import ArmTip2D


# Upper arm length
A 	= 250

# Forearm length
B 	= 250

# Starting position arm straight front
posX 	= 500
posY 	= 0
rotZ	= 0
vel 	= 0
accel 	= 0

shoulder = -90
elbow = 0
wrist = 0

ShoulderAng = 0
ElbowAng = 0

# Screw length per rotation
scrw	= 2 # mm

# Motor default rotations
motor1Zero = 0
motor2Zero = 0

def brutalExit():
	os.kill(os.getpid(), 9)


def xyToJointAngles(x,y):
	# a quich hack to make the math happy
	x -= 0.001
	y += 0.001

	# IK calculation
	ang1 = math.acos( ( (x**2)+(y**2)-(A**2)-(B**2) ) / (2*A*B) )				# elbow joint
	ang2 = math.atan(y/x) + math.asin( (B*math.sin(ang1) ) / (math.sqrt((x**2)+(y**2)))) 	# shoulder joint

	if x < 0:
		ang2 -= math.piorigoX

	return (-math.degrees(ang1), math.degrees(ang2))


# Small visualization:
#--------------------------------------------------------------------------------------------------
def circular_arc(canvas, x, y, r, t0, t1, width):
   		 return canvas.create_arc(x-r, y-r, x+r, y+r, start=t0, extent=t1-t0, style='arc', width=width)

class sim(tk.Tk): #https://stackoverflow.com/questions/6161816/tkinter-how-to-make-tkinter-to-refresh-and-delete-last-lines
	def __init__(self, *args, **kwargs):
		tk.Tk.__init__(self, *args, **kwargs)

		self.size=250
		origoX = 700 - 100
		origoY = 100

		self.title("Arm sim")
		self.w = tk.Canvas(self, width=800, height=600, bg="#456", relief= "sunken", border=10)
		self.w.pack()

		self.w.create_line(0,0,0,0, width=5, fill="#ffc", tags="A")
		self.w.create_line(0,0,0,0, width=3, fill="red",  tags="B")
		self.w.create_line(0,0,0,0, width=2, fill="blue", tags="C")
		self.w.create_oval(0, 0, 0, 0, fill="black", tags="j1")
		self.w.create_oval(0, 0, 0, 0, fill="black", tags="j2")
		self.w.create_oval(0, 0, 0, 0, fill="black", tags="j3")

		# Boundaries
		shoulderMin = -45
		shoulderMax = 15
		elbowMin = -90
		elbowMax = 0
		wristMin = -90
		wristMax = 90

		circular_arc(self.w, origoX, origoY, A+B, 180-shoulderMax, 180-shoulderMin, 2)
		circular_arc(self.w, origoX, origoY, sqrt(A**2+B**2), 210, 210-shoulderMin+shoulderMax, 2)
		circular_arc(self.w, origoX -A*cos(shoulderMax*(pi/180)), origoY -A*sin(shoulderMax*(pi/180)), B, 180-elbowMax-shoulderMax, 180-elbowMin-shoulderMax, 2)
		circular_arc(self.w, origoX -A*cos(shoulderMin*(pi/180)), origoY -A*sin(shoulderMin*(pi/180)), B, 180 -shoulderMin-elbowMax, 180 -shoulderMin-elbowMin, 2)


		e = tk.Button(self,text="Quit", command=self.Quit)
		e.pack()

		self.update_clock()


	def update_clock(self):

		# Origo position on canvas
		ox = 700 - 100
		oy = 100

		shoulder	= IK_node.xyToJointAngles(posX,posY)[1]
		elbow 		= IK_node.xyToJointAngles(posX,posY)[0]
		wrist 		= -IK_node.wristGlobalToJointAngle(rotZ, IK_node.xyToJointAngles(posX,posY)[1], IK_node.xyToJointAngles(posX,posY)[0])
		wrist = rotZ

		# First joint
		degrees = shoulder - 90
		angle = degrees*pi*2/360
#		angle = (ShoulderAng-90)*pi*2/360
		x = ox + self.size*sin(angle)
		y = oy - self.size*cos(angle)
		self.w.coords("A", (ox,oy,x,y))
		self.w.coords("j1", (ox-10,oy-10,ox+10,oy+10))

		# Second joint
		degrees1 = elbow + degrees
		angle1 = degrees1*pi*2/360
#		angle1 = ElbowAng*pi*2/360 + angle
		x1 = x + self.size*sin(angle1)
		y1 = y - self.size*cos(angle1)
		self.w.coords("B", (x,y,x1,y1))
		self.w.coords("j2", (x-7,y-7,x+7,y+7))

		# Third joint
		wristAngleD = wrist +shoulder +elbow -90
		#wristAngleD = -90 -90
		print("on canvas", int(wristAngleD))
		print("on joint", int(wrist))
		#if wrist <= -90:
		#	wristAngleD = shoulder + elbow
		wristAngleR = wristAngleD*pi*2/360
#		wristAngle = ElbowAng*pi*2/360 + angle + angle1
		x2 = x1 + 50*sin(wristAngleD*pi*2/360)
		y2 = y1 - 50*cos(wristAngleD*pi*2/360)
		self.w.coords("C", (x1,y1,x2,y2))
		self.w.coords("j3", (x1-5,y1-5,x1+5,y1+5))

		self.after(100, self.update_clock)

	def Quit(self):
		self.after(700,self.destroy())
		brutalExit()
		exit()


def tipCallback(data):
	global posX
	global posY
	global rotZ
	global shoulder

	posX = data.x
	posY = data.y
	rotZ = data.R




def anglesCallback(data):
	pass


# Main loop ----------------------------------------------------
def main():
	global app
	rospy.init_node('arm_viz')#, anonymous=True)
	rospy.Subscriber("tip_xy_goal", ArmTip2D, tipCallback)
#	rospy.Subscriber("motor_angles_position", MotorAngles, anglesCallback)


	rate = rospy.Rate(10) # 10hz

	app = sim()
	app.mainloop()

	while not rospy.is_shutdown():


		rate.sleep()

if __name__ == '__main__':
	try:
		#initialRot()
		#threading.Thread(target=draw).start()
		main()

	except	KeyboardInterrupt:
		app.destroy()
		quit()
		# brutalExit()
		pass

	except rospy.ROSInterruptException:
		app.destroy()
		quit()
		# brutalExit()
		pass
