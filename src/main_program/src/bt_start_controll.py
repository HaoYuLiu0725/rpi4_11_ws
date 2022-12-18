#!/usr/bin/env python3
# coding: utf-8

import rospy
import serial
import copy
from urllib import response
from main_program.srv import *

port = serial.Serial("/dev/rfcomm0", baudrate=9600)

def main():
	port.reset_input_buffer()
	print(f"Port: '{port._port}' open complete")
	rospy.init_node("py_bt_start_controll")
	running = False
	inVar = 0
	last_inVar = 0
	req = starting
	script = 0
	req._request_class.startStatus = script
	req._request_class.startTrigger = False

	print("Waiting for service...")
	rospy.wait_for_service("/startRunning")
	print("service is good!")

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if(not running):
			print(f"Waiting... || script = {script} || Trigger = {req._request_class.startTrigger}")
			last_inVar = inVar
			inVar = int(port.read_until().decode())
			print(inVar)
			if(inVar != last_inVar):
				if(inVar == 99):
					req._request_class.startTrigger = True
				else:
					script = inVar
					req._request_class.startStatus = script

				print(f"Changed ! || script = {script} || Trigger = {req._request_class.startTrigger}")
				
				try:
					client = rospy.ServiceProxy("/startRunning", starting)
					client(req)
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


