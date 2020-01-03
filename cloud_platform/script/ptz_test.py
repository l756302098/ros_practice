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
from yidamsg.msg import Yida_pose
from yidamsg.msg import transfer
from yidamsg.msg import InspectedResult
from yidamsg.msg import Detect_Result

g_anglez = 0.0
x_camera = 0.0
y_camera = 0.0
z_camera = 0.0

def read_config():
	file = open("/home/muyangren/catkin_ws/src/cloud_platform/script/config.cfg") 
	while True:
		line = file.readline()
		if not line:
			break
		line_param = []
		line_param = line.split(':')
		cloudplat_param = []
		cloudplat_param = line_param[1].split(',')
		global xy_min, xy_max, z_min, z_max
		if line_param[0] == 'xy':
			xy_min = int(cloudplat_param[0])
			xy_max = int(cloudplat_param[1])
			print("xy_min: " + str(int(xy_min)), "xy_max: " + str(int(xy_max)))
		elif line_param[0] == 'z':
			z_min = int(cloudplat_param[0])
			z_max = int(cloudplat_param[1])
			print("z_min: " + str(int(z_min)), "z_max: " + str(int(z_max)))
		pass # do something
	file.close()

def get_xydegree(x_camera, y_camera, z_camera, x_device, y_device, z_device):
	x_distance = float(x_device) - float(x_camera)
	y_distance = float(y_device) - float(y_camera)
	z_distance = float(z_device) - float(z_camera)
	if x_distance == 0.0:
		x_distance = 0.00001
	devicedegree = math.atan(y_distance / x_distance)
	devicedegree_final = math.degrees(devicedegree)
	print('=====get_xydegree result is %s.====='% devicedegree_final)
	return devicedegree_final

def get_zdegree(x_camera, y_camera, z_camera, x_device, y_device, z_device):
	x_distance = float(x_device) - float(x_camera)
	y_distance = float(y_device) - float(y_camera)
	z_distance = float(z_device) - float(z_camera)
	zxy_distance = math.sqrt(math.pow(x_distance, 2) + math.pow(y_distance, 2))
	if zxy_distance == 0.0:
		zxy_distance = 0.00001
	zdegree = math.atan(z_distance / zxy_distance)
	zdegree_final = math.degrees(zdegree)
	print('=====get_zdegree result is %s.====='% zdegree_final)
	return zdegree_final

def callback_pose(data):
	print('------------callback_pose-------------')
	'''
	trans = [0,0,0]
	try:
		(trans, rot) = listener.lookupTransform('/map', '/hk_link', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass
		print('************get trans data error!************')
	#print(data.x, data.y, data.z, data.anglex, data.angley, data.anglez)

	global x_camera,y_camera,z_camera,g_anglez
	#x_camera = data.x
	#y_camera = data.z
	#z_camera = data.y
	x_camera = trans[0]
	y_camera = trans[1]
	z_camera = trans[2]

	g_anglez = data.anglez
	'''
	global x_camera,y_camera,z_camera,g_anglez
	x_camera = 0.0
	y_camera = 0.0
	z_camera = 0.0
	g_anglez = 0.0

def callback_transfer(data):
	str_devicepoint = data.data
	transferpub_msg = []
	transferpub_msg = str_devicepoint.split('/')
	if(transferpub_msg[3] == '2'):
		device_point = transferpub_msg[4].split(',')
		x_device = float(device_point[0])
		y_device = float(device_point[1])
		z_device = float(device_point[2])

		global x_camera, y_camera, z_camera, g_anglez
		print('=====x_device, y_device, z_device value is {0}, {1}, {2}.====='.format(x_device, y_device, z_device))
		print('=====x_camera, y_camera, z_camera value is {0}, {1}, {2}.====='.format(x_camera, y_camera, z_camera))
		xydegree = get_xydegree(x_camera, y_camera, z_camera, x_device, y_device, z_device)
		zdegree = get_zdegree(x_camera, y_camera, z_camera, x_device, y_device, z_device)
		
		#设置水平值
		cloudplat_control = CloudPlatControl()
		cloudplat_control.id = 0
		cloudplat_control.action = 0
		cloudplat_control.type = 0
		cloudplat_control.value = 0
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		val_panpos = getresult.result;
		if x_device >= x_camera:
			finalx_set = math.degrees(g_anglez) - xydegree
		elif ((x_device < x_camera) and (y_device >= y_camera)):
			finalx_set = -180 - xydegree + math.degrees(g_anglez)
		elif ((x_device < x_camera) and (y_device <= y_camera)):
			finalx_set = 180 - xydegree + math.degrees(g_anglez)
		else:
			pass
		if finalx_set > 0:
			finalx_set = val_panpos + abs(finalx_set * 100)
			if finalx_set > 35999:
				finalx_set = finalx_set - 35999
		elif finalx_set < 0:
			finalx_set = val_panpos - abs(finalx_set * 100)
			if finalx_set < 0:
				finalx_set = 35999 + finalx_set

		#设置垂直值
		cloudplat_control.id = 0
		cloudplat_control.action = 0
		cloudplat_control.type = 1
		cloudplat_control.value = 0
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		val_tiltpos = getresult.result;
		finalz_set = val_tiltpos - zdegree * 100

		if z_device >= z_camera:
			if ((finalz_set >= 9000) and (finalz_set < 27000)):
				finalz_set = 27000
			elif finalz_set < 0:
				finalz_set = 35999 + finalz_set
		if z_device < z_camera:
			if ((finalz_set > 9000) and (finalz_set <= 27000)):
				finalz_set = 9000
			elif finalz_set >35999:
				finalz_set = finalz_set -35999

		print("val_panpos: " + str(int(val_panpos)), "val_tiltpos: " + str(int(val_tiltpos)))
		print("finalx_set: " + str(int(finalx_set)), "finalz_set: " + str(int(finalz_set)))
		cloudplat_control.id = 0
		cloudplat_control.action = 1
		cloudplat_control.type = 0
		cloudplat_control.value = int(finalx_set)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		cloudplat_control.id = 0
		cloudplat_control.action = 1
		cloudplat_control.type = 1
		cloudplat_control.value = int(finalz_set)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(5)
		
		#转到预置位1
		cloudplat_control = CloudPlatControl()
		cloudplat_control.id = 0
		cloudplat_control.action = 0
		cloudplat_control.type = 3
		cloudplat_control.value = 1
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		print('=====cloudplat_control result is %s.====='% getresult.result)
		time.sleep(3)

def set_position_first():
	cloudplat_control = CloudPlatControl()
	#设置水平绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 0
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置垂直绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 1
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置变倍绝对值为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 2
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(5)

def ircameraptz_init():
	cloudplat_control = CloudPlatControl()
	#设置水平绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 0
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置垂直绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 1
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置变倍绝对值为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 2
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(5)
	'''
	#设置聚焦位置为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 4
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(5)
	'''
	'''
	#设置预置位1
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 3
	cloudplat_control.value = 1
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(5)
	'''
	#设置水平绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 0
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置垂直绝对角度为0
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 1
	cloudplat_control.value = 380
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	#设置变倍绝对值为0x4000
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 2
	cloudplat_control.value = 0x4000
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(10)
	
	'''
	#设置聚焦位置为0x3626
	cloudplat_control.id = 0
	cloudplat_control.action = 1
	cloudplat_control.type = 4
	#cloudplat_control.value = 0x3626
	#cloudplat_control.value = 0xA89F
	cloudplat_control.value = 0x2B1D
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(10)
	'''
	'''
	cloudplat_control.id = 0
	cloudplat_control.action = 0
	cloudplat_control.type = 4
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	print("*************is {0}".format(getresult.result))
	time.sleep(5)
	'''
	'''	
	#转到预置位1
	cloudplat_control = CloudPlatControl()
	cloudplat_control.id = 0
	cloudplat_control.action = 0
	cloudplat_control.type = 3
	cloudplat_control.value = 1
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	print('=====cloudplat_control result is %s.====='% getresult.result)
	time.sleep(5)
	'''
	set_position_first()


if __name__ == '__main__':
	rospy.init_node('infrared_cloudplat', anonymous=True)
	read_config()

	rospy.Subscriber("/robot_pose", Yida_pose, callback_pose, queue_size=1)
	rospy.Subscriber("/transfer_pub", transfer, callback_transfer, queue_size=1)
	meterflag_pub = rospy.Publisher("/meter_flag",String, queue_size=1)
	infrared_result_pub = rospy.Publisher("/detect_result", InspectedResult, queue_size=1)
	Ptz_fun = rospy.ServiceProxy('cloudplatform_control', CloudPlatControl)
	ircameraptz_init()
	rospy.spin()
