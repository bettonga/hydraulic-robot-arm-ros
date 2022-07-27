#!/usr/bin/env python3

import serial # pyserial
import time
import os
import threading
import math

# Ros imports
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

ser = serial.Serial()
connected = 0
JOINT_GOAL = [0,0,0]
JOINT_POS = [0,0,0]

# Init joystick axes
LY = LX = RX = 0
buttons = [0,0,0,0,0,0,0,0,0,0,0]

# Instant exit function
def brutalExit():
	os.kill(os.getpid(), 9)

# Initialize and open serial port
def serSetup():
	global ser
	ser.port = '/dev/ttyACM0'
	ser.baudrate = 115200
	ser.bytesize = serial.EIGHTBITS
	ser.parity = serial.PARITY_NONE
	ser.stopbits = serial.STOPBITS_ONE
	ser.timeout = 1
	ser.open()

# Pass command mesage from ros to serial
MSG_ANG = ["ANG 0 0\n", "ANG 1 0\n", "ANG 2 0\n"]
def anglesGoalCallback(data):
	global MSG_ANG
	global JOINT_GOAL
	global ser
	new = data.data
	if new != JOINT_GOAL:
		JOINT_GOAL = new
	MSG_ANG = ["ANG {} {}\n".format(i, JOINT_GOAL[i]) for i in range(3)]

MSG_SPD = ["SPD 0 2048\n", "SPD 1 2048\n", "SPD 2 2048\n"]
def joyCallback(data):
	global MSG_SPD
	global vel
	global accel
	global LY, RY, cross
	global buttons

	axes = data.axes

	LY = axes[1]
	LX = axes[0]
	RX = axes[2]
	vel = int(max(abs(LX),abs(LY))*15)
	buttons = data.buttons
	MSG_SPD = ["SPD 0 {}\n".format(int((0.6*LY+1.0)*2048)),
			   "SPD 1 {}\n".format(int((0.6*LX+1.0)*2048)),
			   "SPD 2 {}\n".format(int((0.6*RX+1.0)*2048))]

# Mainloop
def main():
	global ser
	global JOINT_POS
	spd_mode = True
	old_state = 0
	while not rospy.is_shutdown():

		if not ser.isOpen():
			serSetup()
		else:
			# select SPD or ANG mode
			try:
				if buttons[4]>0.5 and old_state<0.5:
					spd_mode = not spd_mode
				old_state = buttons[4]
			except:
				pass

			# send commands to the motors
			MSG = MSG_SPD if spd_mode else MSG_ANG
			for msg in MSG:
				ser.write(msg.encode())

			# receive the sensors data
			received = ser.readline().decode('utf-8').replace("\r\n","").split(" ")
			try:
				JOINT_POS = [min(16384, max(-16384, int(received[i]) )) for i in range(3)]
				# JOINT_POS = [min(180,max(-180,int(received[1]))), min(180,max(-180,int(received[2]))), min(180,max(-180,int(received[0])))]
			except:
				pass
			angles_out.data = JOINT_POS		# [shoulder, elbow, wrist (, speed, accel)]
			joint_angles_position.publish(angles_out)

		rate.sleep()


if __name__ == '__main__':
	try:

		rospy.init_node('motor_node')#, anonymous=True)
		rate = rospy.Rate(100) # 100hz (same as the main loop of pico)
		serSetup()

		rospy.Subscriber("/joy", Joy, joyCallback)
		rospy.Subscriber("joint_angles_goal", Int16MultiArray, anglesGoalCallback)
		joint_angles_position = rospy.Publisher('joint_angles_position', Int16MultiArray, queue_size=10)
		angles_out = Int16MultiArray()	# [shoulder, elbow, wrist (, speed, accel)]

		main()

	except KeyboardInterrupt:
		# ser.close()
		brutalExit()

	except rospy.ROSInterruptException:
		brutalExit()
