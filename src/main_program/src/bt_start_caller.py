#!/usr/bin/env python3
# coding: utf-8

import rospy
import serial
import copy
from urllib import response
from main_program.srv import *
from std_msgs.msg import Bool

port = serial.Serial("/dev/rfcomm0", baudrate=9600)
req = startingRequest()
running = False

def CallBackFunction(msg):
	rospy.loginfo(f"script_finish: {msg.data}")
	global running
	if(msg.data):
		running = False
		req.startTrigger = False
		port.write(bytes(str(69),'utf-8'))

def main():
	port.reset_input_buffer()
	print(f"Port: '{port._port}' open complete")
	rospy.init_node("py_bt_start_controll")
	inVar = 0
	last_inVar = 0
	script = 0
	global running
	req.startTrigger = False
	req.startStatus = script

	rospy.Subscriber("/script_finish", Bool, CallBackFunction)
	print("Waiting for service...")
	rospy.wait_for_service("/startRunning")
	print("service is good!")

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if(not running):
			print(f"Waiting... || Status = {req.startStatus} || Trigger = {req.startTrigger}")
			last_inVar = inVar
			inVar = int(port.read_until().decode())
			print(inVar)
			if(inVar != last_inVar):
				if(inVar == 99):
					req.startTrigger = True
					running = True
				else:
					script = inVar
					req.startStatus = script

				print(f"Changed ! || Status = {req.startStatus} || Trigger = {req.startTrigger}")
				if(running):
					print(f"Running !")
				
				try:
					client = rospy.ServiceProxy("/startRunning", starting)
					resp = client(req)
					# rospy.loginfo("Message from server: %s" %response.feedback)
				except rospy.ServiceException as e:
					rospy.logwarn("Service call failed: %s" %e)
		
		rate.sleep()		

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		port.close()
