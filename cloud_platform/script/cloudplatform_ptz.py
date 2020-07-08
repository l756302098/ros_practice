#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import sys
import time
import thread
import threading
import rospy
import serial
import cv2
import math
import tf
import numpy as np
from std_msgs.msg import String
from cloud_platform.srv import *

platform_id = -1
ser_dev = None
cmd_cloudplat = [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]

def get_sumcs(cmdin):
	result = (cmdin[1] + cmdin[2] + cmdin[3] + cmdin[4] + cmdin[5]) % 256;
	return result

def init_usbdev(cloudplatform_id):
	print('-----init usb device-----')
	if cloudplatform_id == 0:
		try:
			ser_dev = serial.Serial('/dev/nr_cloudplatform', 9600)
			print('the device id is 0')
			return ser_dev
		except serial.serialutil.SerialException as e:
			print('Exception: Cloud platform device connect error!')
		finally:
			pass
	elif cloudplatform_id == 1:
		try:
			#ser_dev = serial.Serial('/dev/visible_cloudplatform', 2400)
			ser_dev = serial.Serial('/dev/nr_cloudplatform', 9600)
			print('the device id is 1')
			return ser_dev
		except serial.serialutil.SerialException as e:
			print('Exception: Cloud platform device connect error!')
		finally:
			pass
	elif cloudplatform_id == 2:
		try:
			#ser_dev = serial.Serial('/dev/infrared_cloudplatform', 2400)
			ser_dev = serial.Serial('/dev/nr_cloudplatform', 9600)
			print('the device id is 2')
			return ser_dev
		except serial.serialutil.SerialException as e:
			print('Exception: Cloud platform device connect error!')
		finally:
			pass
	else:
		print('Device id error!')

def cmd_serialcontrol(ser, cmd_cloudplat, action, typea):
	n = 0
	while n == 0:
		ser.write(cmd_cloudplat)
		print('cmd_cloudplat:', cmd_cloudplat)
		time.sleep(0.5)
		try:
			data = ''
			n = ser.inWaiting()
			print('the n value is: ', n)
			if n:
				data = data + ser.read(n)
				for l in xrange(len(data)):
					pass
					#print '%02X' % ord(data[l])
					#print(ord(data[l]))
			if ((action == 0) and (typea != 3)):
				return ord(data[4])*256 + ord(data[5])
			else:
				return 1
		except IOError:
			print "Error!"

def callback_cloudplatform(req):
	id_msg = req.id
	global platform_id,ser_dev
	if id_msg != platform_id:
		platform_id = req.id
		ser_dev = init_usbdev(req.id)

	if req.id == 0:
		if (req.action == 0 and req.type == 0):
			print('---获取水平绝对角度---')
			cmd_cloudplat[2] = 0x47
			cmd_cloudplat[3] = 0x00
		elif (req.action == 0 and req.type == 1):
			print('---获取垂直绝对角度---')
			cmd_cloudplat[2] = 0x49
			cmd_cloudplat[3] = 0x00
		elif (req.action == 0 and req.type == 2):
			print('---获取变倍绝对值---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x55
		elif (req.action == 0 and req.type == 3):
			print('---转到预置位---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x07
		elif (req.action == 0 and req.type == 4):
			print('---获取聚焦位置---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x57
		elif (req.action == 1 and req.type == 0):
			print('---设置水平绝对角度---')
			cmd_cloudplat[2] = 0x41
			cmd_cloudplat[3] = 0x00
		elif (req.action == 1 and req.type == 1):
			print('---设置垂直绝对角度---')
			cmd_cloudplat[2] = 0x42
			cmd_cloudplat[3] = 0x00
		elif (req.action == 1 and req.type == 2):
			print('---设置变倍绝对值---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x5D
		elif (req.action == 1 and req.type == 3):
			print('---设置预置位---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x03
		elif (req.action == 1 and req.type == 4):
			print('---设置聚焦位置---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x5F
		else:
			print('Action or type error!')
		cmd_cloudplat[4] = req.value / 256
		cmd_cloudplat[5] = req.value % 256
		cmd_cloudplat[6] = get_sumcs(cmd_cloudplat)
		result_value = cmd_serialcontrol(ser_dev, cmd_cloudplat, req.action, req.type)

	if req.id == 1 or req.id == 2:
		if (req.action == 0 and req.type == 0):
			print('---获取水平绝对角度---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x51
		elif (req.action == 0 and req.type == 1):
			print('---获取垂直绝对角度---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x53
		elif (req.action == 0 and req.type == 2):
			print('---获取变倍绝对值---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x55
		elif (req.action == 0 and req.type == 3):
			print('---转到预置位---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x07
		elif (req.action == 1 and req.type == 0):
			print('---设置水平绝对角度---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x4B
		elif (req.action == 1 and req.type == 1):
			print('---设置垂直绝对角度---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x4D
		elif (req.action == 1 and req.type == 2):
			print('---设置变倍绝对值---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x4F
		elif (req.action == 1 and req.type == 3):
			print('---设置预置位---')
			cmd_cloudplat[2] = 0x00
			cmd_cloudplat[3] = 0x03
		else:
			print('Action or type error!')
		cmd_cloudplat[4] = req.value / 256
		cmd_cloudplat[5] = req.value % 256
		cmd_cloudplat[6] = get_sumcs(cmd_cloudplat)
		result_value = cmd_serialcontrol(ser_dev, cmd_cloudplat, req.action, req.type)

	return result_value

if __name__ == '__main__':
	rospy.init_node('cloudplat', anonymous=True)
	s = rospy.Service('cloudplatform_control', CloudPlatControl, callback_cloudplatform)
	rospy.spin()
