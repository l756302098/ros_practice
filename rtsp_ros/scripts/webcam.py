#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import cv2, rospy, time
import numpy as np
from sensor_msgs.msg import CompressedImage

if __name__=='__main__':
    rospy.init_node("webcam", anonymous=True)
    name = rospy.get_name()    
    topic_name = rospy.get_param(name+"/topic_name")
    image_width = rospy.get_param(name+'/image_width')    
    image_height = rospy.get_param(name+"/image_height")
    frequency = rospy.get_param(name+"/frequency")
    camera_id = rospy.get_param(name+"/camera_id")
    publisher = rospy.Publisher(topic_name, CompressedImage, queue_size=1)
    rate = rospy.Rate(frequency)
    cap = cv2.VideoCapture(camera_id)
    # set CV_CAP_PROP_FRAME_WIDTH
    cap.set(3, image_width)
    # set CV_CAP_PROP_FRAME_HEIGHT
    cap.set(4, image_height) 
    # set frequency
    cap.set(5, frequency) 

    cnt = 0
    while not rospy.is_shutdown():
        cnt = cnt + 1
        t1 = time.clock()
        # get a frame
        ret, frame = cap.read()
        image = frame
        #image = cv2.resize(frame, (640, 480))
        image = cv2.resize(frame, (1280, 720))
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        publisher.publish(msg)
        # show a frame
        cv2.imshow(str(camera_id), image)
        cv2.waitKey(1)
        t2 = time.clock()
        print 'time is: ', t2-t1
        #rate.sleep()
