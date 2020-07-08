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
from yidamsg.msg import yuntai_control
from yidamsg.msg import yuntai_preset_control
from yidamsg.msg import yuntai_reset_control

def read_config():
	file = open("/home/zdyd/workspace/src/visiblecamera_ptz/script/visible_camera_config.cfg") 
	#file = open("visible_camera_config.cfg") 
	while True:
		line = file.readline()
		if not line:
			break
		line_param = []
		line_param = line.split(':')

		global camera_id, camera_ip, camera_username, camera_password
		global camera_image_width, camera_image_height, camera_cmos_width, camera_cmos_height
		global camera_pixel_size, camera_focus_number, camera_focus_mm, camera_focus_value
		global x_min_degree, x_max_degree, z_origin_degree, z_min_degree, z_max_degree
		global offset_xy

		if line_param[0] == 'camera_id':
			camera_id = int(line_param[1])
			print("camera_id value is: " + str(camera_id) + " type is: " + str(type(camera_id)))
		elif line_param[0] == 'camera_ip':
			camera_ip = line_param[1]
			print("camera_ip value is: " + camera_ip + "  type is: " + str(type(camera_ip)))
		elif line_param[0] == 'camera_username':
			camera_username = line_param[1]
			print("camera_username value is: " + camera_username + "  type is:" + str(type(camera_username)))
		elif line_param[0] == 'camera_password':
			camera_password = line_param[1]
			print("camera_password value is: " + camera_password + "  type is:" + str(type(camera_password)))
			pass
		elif line_param[0] == 'camera_image_width':
			camera_image_width = int(line_param[1])
			print("camera_image_width value is: " + str(camera_image_width) + "  type is:" + str(type(camera_image_width)))
		elif line_param[0] == 'camera_image_height':
			camera_image_height = int(line_param[1])
			print("camera_image_height value is: " + str(camera_image_height) + "  type is:" + str(type(camera_image_height)))
		elif line_param[0] == 'camera_cmos_width':
			camera_cmos_width = float(line_param[1])
			print("camera_cmos_width value is: " + str(camera_cmos_width) + "  type is:" + str(type(camera_cmos_width)))
		elif line_param[0] == 'camera_cmos_height':
			camera_cmos_height = float(line_param[1])
			print("camera_cmos_height value is: " + str(camera_cmos_height) + "  type is:" + str(type(camera_cmos_height)))
		elif line_param[0] == 'camera_pixel_size':
			camera_pixel_size = float(line_param[1])
			print("camera_pixel_size value is: " + str(camera_pixel_size) + "  type is:" + str(type(camera_pixel_size)))
		elif line_param[0] == 'camera_focus_number':
			camera_focus_number = int(line_param[1])
			print("camera_focus_number value is: " + str(camera_focus_number) + "  type is:" + str(type(camera_focus_number)))
			pass
		elif line_param[0] == 'camera_focus_mm':
			camera_focus_mm = line_param[1].split(',')
			camera_focus_mm = map(eval, camera_focus_mm)
			print("camera_focus_mm value is: " + str(camera_focus_mm) + "  type is:" + str(type(camera_focus_mm)))
		elif line_param[0] == 'camera_focus_value':
			camera_focus_value = line_param[1].split(',')
			camera_focus_value = map(eval, camera_focus_value)
			print("camera_focus_value value is: " + str(camera_focus_value) + "  type is:" + str(type(camera_focus_value)))
		elif line_param[0] == 'xyz_degree':
			cloudplat_param = []
			cloudplat_param = line_param[1].split(',')
			x_min_degree = int(cloudplat_param[0])
			x_max_degree = int(cloudplat_param[1])
			print("x_min_degree: " + str(int(x_min_degree)), "x_max_degree: " + str(int(x_max_degree)))
			z_origin_degree = int(cloudplat_param[2])
			z_max_degree = int(cloudplat_param[3])
			z_min_degree = int(cloudplat_param[4])
			print("z_origin_degree: " + str(int(z_origin_degree)), "z_max_degree: "  \
				+ str(int(z_max_degree)), "z_min_degree: " + str(int(z_min_degree)))
		elif line_param[0] == 'offset_xy':
			 offset_xy = int(line_param[1])
			 print("offset_xy value is: " + str(offset_xy) + "  type is:" + str(type(offset_xy)))
	file.close()

def callback_yuntaictrl(data):
	print(data.direction)
	print(data.number)
	print(data.step)

	diff_value = 0
	camera_id = data.number

	if data.step == 1:
		#1 degree
		diff_value = 100
	elif data.step == 2:
		#10 degree 
		diff_value = 1000
	elif data.step == 3:
		#50 degree
		diff_value = 5000

	cloudplat_control = CloudPlatControl()
	#get val_panpos
	cloudplat_control.id = camera_id
	cloudplat_control.action = 0
	cloudplat_control.type = 0
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	val_panpos = getresult.result;
	time.sleep(0.5)
	
	#get val_tiltpos
	cloudplat_control = CloudPlatControl()
	cloudplat_control.id = camera_id
	cloudplat_control.action = 0
	cloudplat_control.type = 1
	cloudplat_control.value = 0
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	val_tiltpos = getresult.result;
	time.sleep(0.5)

	if data.direction == "right":
		val_panpos = val_panpos + diff_value
		if val_panpos > 35999:
			val_panpos = val_panpos - 35999
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 0
		cloudplat_control.value = int(val_panpos)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)
	elif data.direction == "left":
		val_panpos = val_panpos - diff_value
		if val_panpos < 0:
			val_panpos = 35999 + val_panpos
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 0
		cloudplat_control.value = int(val_panpos)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)
	#0=horizontally 90=straight down 270=straight up
	#range 0-9000 and 27000-35999		
	elif data.direction == "up":
		val_tiltpos = val_tiltpos - diff_value
		if ((val_tiltpos >= z_min_degree) and (val_tiltpos < z_max_degree)):
			val_tiltpos = z_max_degree
		elif val_tiltpos < 0:
			val_tiltpos = z_origin_degree + val_tiltpos
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 1
		cloudplat_control.value = int(val_tiltpos)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)
	elif data.direction == "down":
		val_tiltpos = val_tiltpos + diff_value
		if ((val_tiltpos > z_min_degree) and (val_tiltpos <= z_max_degree)):
			val_tiltpos = z_min_degree
		elif val_tiltpos > z_origin_degree:
			val_tiltpos = val_tiltpos - z_origin_degree
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 1
		cloudplat_control.value = int(val_tiltpos)
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)

def callback_yuntaipreset(data):
	print(data.id)
	print(data.horizontal)
	print(data.vertical)
	print(data.focus)

	global camera_focus_mm, camera_focus_value, camera_focus_number
	camera_id = data.id
	val_panpos = data.horizontal * 100
	if data.vertical <= 0:
		val_tiltpos = -100 * data.vertical
	elif data.vertical >= 0:
		val_tiltpos = 35999 - data.vertical * 100
	
	if data.focus < 0:
		data.focus = 0
	elif data.focus > 29:
		data.focus = 29
	val_zoom = camera_focus_value[data.focus]

	cloudplat_control = CloudPlatControl()
	cloudplat_control.id = camera_id
	cloudplat_control.action = 1
	cloudplat_control.type = 0
	cloudplat_control.value = int(val_panpos)
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(0.5)
	cloudplat_control.id = camera_id
	cloudplat_control.action = 1
	cloudplat_control.type = 1
	cloudplat_control.value = int(val_tiltpos)
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(0.5)
	#设置变倍值
	cloudplat_control.id = camera_id
	cloudplat_control.action = 1
	cloudplat_control.type = 2
	cloudplat_control.value = val_zoom
	getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
	time.sleep(2)

def callback_yuntaireset(data):
	print(data.id)
	print(data.data)

	cloudplat_control = CloudPlatControl()
	camera_id = data.id
	if data.data == 1:
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 0
		cloudplat_control.value = 0
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 1
		cloudplat_control.value = 0
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(0.5)
		#设置变倍值
		cloudplat_control.id = camera_id
		cloudplat_control.action = 1
		cloudplat_control.type = 2
		cloudplat_control.value = 8
		getresult=Ptz_fun(cloudplat_control.id,cloudplat_control.action,cloudplat_control.type,cloudplat_control.value)
		time.sleep(2)

if __name__ == '__main__':
	rospy.init_node('ptz_fromwin', anonymous=True)
	read_config()

	rospy.Subscriber("/yunTai_ctrl", yuntai_control, callback_yuntaictrl, queue_size=1)
	rospy.Subscriber("/preset", yuntai_preset_control, callback_yuntaipreset, queue_size=1)
	rospy.Subscriber("/preset_reset", yuntai_reset_control, callback_yuntaireset, queue_size=1)
	Ptz_fun = rospy.ServiceProxy('cloudplatform_control', CloudPlatControl)
	rospy.spin()
