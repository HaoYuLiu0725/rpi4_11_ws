#!/usr/bin/env python3
# coding: utf-8

import rospy
import serial
# from urllib import response
# from main_program.srv import *

port = serial.Serial("/dev/rfcomm0", baudrate=9600)

def main():
	port.reset_input_buffer()
	# rospy.init_node("py_bt_start_controll")
	# rospy.wait_for_service("/startRunning")
	script = 0
	# request = starting
	# request._request_class.startStatus = script
	# rate = rospy.Rate(10)
	# while not rospy.is_shutdown():
		# script = port.read_until().decode()
		# print(script)
		# print(type(script))
		# try:
		# 	client = rospy.ServiceProxy("name_and_age", starting)
		# 	response = client()
		# 	rospy.loginfo("Message from server: %s" %response.feedback)
		# except rospy.ServiceException as e:
		# 	rospy.logwarn("Service call failed: %s" %e)
		# rate.sleep()
	while True:
		script = int(port.read_until().decode())
		print(script)
		# print(type(script))

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		port.close()


