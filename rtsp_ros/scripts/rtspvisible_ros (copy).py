#!/usr/bin/env python
# *-* coding:utf-8 *-*

import rospy
import cv2
import threading
import numpy as np
import os
from sensor_msgs.msg import CompressedImage

camera_id = 0
camera_ip = '192.168.18.64'
camera_username = 'admin'
camera_password = '1234qwer'
camera_image_width = 1280
camera_image_height = 960
camera_cmos_width = 5.28
camera_cmos_height = 2.97
camera_pixel_size = 3.75
camera_focus_number = 0
camera_focus_mm = []
camera_focus_value = []
offset_xy = 1
device_id = ''


def read_config():
    file = open(
        "/home/zdyd/workspace/src/visiblecamera_ptz/script/visible_camera_config.cfg")
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
            print("camera_id value is: " + str(camera_id) +
                  " type is: " + str(type(camera_id)))
        elif line_param[0] == 'camera_ip':
            camera_ip = line_param[1]
            print("camera_ip value is: " + camera_ip +
                  "  type is: " + str(type(camera_ip)))
        elif line_param[0] == 'camera_username':
            camera_username = line_param[1]
            print("camera_username value is: " + camera_username +
                  "  type is:" + str(type(camera_username)))
        elif line_param[0] == 'camera_password':
            camera_password = line_param[1]
            print("camera_password value is: " + camera_password +
                  "  type is:" + str(type(camera_password)))
            pass
        elif line_param[0] == 'camera_image_width':
            camera_image_width = int(line_param[1])
            print("camera_image_width value is: " + str(camera_image_width) +
                  "  type is:" + str(type(camera_image_width)))
        elif line_param[0] == 'camera_image_height':
            camera_image_height = int(line_param[1])
            print("camera_image_height value is: " + str(camera_image_height) +
                  "  type is:" + str(type(camera_image_height)))
        elif line_param[0] == 'camera_cmos_width':
            camera_cmos_width = float(line_param[1])
            print("camera_cmos_width value is: " + str(camera_cmos_width) +
                  "  type is:" + str(type(camera_cmos_width)))
        elif line_param[0] == 'camera_cmos_height':
            camera_cmos_height = float(line_param[1])
            print("camera_cmos_height value is: " + str(camera_cmos_height) +
                  "  type is:" + str(type(camera_cmos_height)))
        elif line_param[0] == 'camera_pixel_size':
            camera_pixel_size = float(line_param[1])
            print("camera_pixel_size value is: " + str(camera_pixel_size) +
                  "  type is:" + str(type(camera_pixel_size)))
        elif line_param[0] == 'camera_focus_number':
            camera_focus_number = int(line_param[1])
            print("camera_focus_number value is: " + str(camera_focus_number) +
                  "  type is:" + str(type(camera_focus_number)))
            pass
        elif line_param[0] == 'camera_focus_mm':
            camera_focus_mm = line_param[1].split(',')
            camera_focus_mm = map(eval, camera_focus_mm)
            print("camera_focus_mm value is: " + str(camera_focus_mm) +
                  "  type is:" + str(type(camera_focus_mm)))
        elif line_param[0] == 'camera_focus_value':
            camera_focus_value = line_param[1].split(',')
            camera_focus_value = map(eval, camera_focus_value)
            print("camera_focus_value value is: " + str(camera_focus_value) +
                  "  type is:" + str(type(camera_focus_value)))
        elif line_param[0] == 'xyz_degree':
            cloudplat_param = []
            cloudplat_param = line_param[1].split(',')
            x_min_degree = int(cloudplat_param[0])
            x_max_degree = int(cloudplat_param[1])
            print("x_min_degree: " + str(int(x_min_degree)),
                  "x_max_degree: " + str(int(x_max_degree)))
            z_origin_degree = int(cloudplat_param[2])
            z_max_degree = int(cloudplat_param[3])
            z_min_degree = int(cloudplat_param[4])
            print("z_origin_degree: " + str(int(z_origin_degree)), "z_max_degree: "
                  + str(int(z_max_degree)), "z_min_degree: " + str(int(z_min_degree)))
        elif line_param[0] == 'offset_xy':
            offset_xy = int(line_param[1])
            print("offset_xy value is: " + str(offset_xy) +
                  "  type is:" + str(type(offset_xy)))
    file.close()


def thread_visiblevideostream():
    print('thread %s is runing...' % threading.current_thread().name)
    global camera_ip
    global camera_username
    global camera_password
    global camera_image_width
    global camera_image_height
    videocap_str = 'rtsp://' + str(camera_username) + ':' + str(
        camera_password) + '@' + str(camera_ip) + ':554/h264/ch1/main/av_stream'
    print(videocap_str, type(videocap_str))

    rs = os.system('ping -c 1 -w 1 %s' % camera_ip)
    if rs:
        print('ping %s is fail, device is not connect!' % camera_ip)
        return 0
    else:
        print('ping %s is ok, device is connect!' % camera_ip)

    capture_stream = cv2.VideoCapture(videocap_str)
    while capture_stream.isOpened() == False:
        print('capture_stream fail to open!')
        capture_stream = cv2.VideoCapture(videocap_str)
    while True:
        framenum_stream = 1
        while(capture_stream.isOpened()):
            _, frame_stream = capture_stream.read()
            if (frame_stream == None):
                print('frame_stream is none')
            else:
                print(frame_stream.size)
            if (frame_stream != None) and (frame_stream.size > 0):
                framenum_stream += 1
                msg = CompressedImage()
                if framenum_stream % 3 == 0:
                    msg.header.stamp = rospy.Time.now()
                    msg.format = "jpeg"
                    msg.data = np.array(cv2.imencode(
                        '.jpg', frame_stream)[1]).tostring()
                    print('publish image')
                    visible_videostream_pub.publish(msg)
                    framenum_stream = 0

                if cv2.waitKey(1) == ord('q'):
                    break

        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('visible_video', anonymous=True)

    read_config()
    t_streamdev = threading.Thread(
        target=thread_visiblevideostream, name='VisibleStreamThread')
    t_streamdev.start()

    visible_videostream_pub = rospy.Publisher(
        "/visible/image_proc/compressed", CompressedImage, queue_size=1)
