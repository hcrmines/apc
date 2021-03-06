#!/usr/bin/env python

from apc.srv import *
from apc.msg import *

import rospy
import os
import cv2
import scipy.io as sio
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

class Recognizer:
    def __init__(self):
        self.bridge = CvBridge()

        self.cameras = [None,None,None]
        self.cloud = None

        self.cv_image = None

        self.mask_3d = -1 * np.ones((480, 640))
        self.mask_2d = -1 * np.ones((400, 640))
        self.categories = []
        self.scores = []
        self.centroids_x = [];
        self.centroids_y = [];
        self.angles_ma = [];
        self.angles_mi = [];

        self.count = 0
        self.cam_num = -1

        self.pub_mask = rospy.Publisher('/apc/recognition_mask_image',
                                        Image, queue_size = 1)
        self.pub_rec_info = rospy.Publisher('/apc/recognition_information',
                                            MatInfo, queue_size = 1)
        self.pub_cloud = rospy.Publisher('/apc/recognition_pcl',
                                         PointCloud2, queue_size = 1)

        rospy.Subscriber("/camera/depth/points",
                         PointCloud2, self.update_point_cloud)
        rospy.Subscriber("/camera/rgb/image_raw",
                         Image, self.update_kinect)
        rospy.Subscriber("/cameras/right_hand_camera/image",
                         Image, self.update_right)
        rospy.Subscriber("/cameras/left_hand_camera/image",
                         Image, self.update_left)

        self.do_recognition = False

    def update_kinect(self, update):
        self.cameras[0] = update
        if self.do_recognition:
            self.recognize()

    def update_left(self, update):
        self.cameras[1] = update

    def update_right(self, update):
        self.cameras[2] = update

    def update_point_cloud(self, update):
        # print "need correct topic name!"
        self.cloud = update

    def recognize(self):
        self.pub_cloud.publish(self.cloud)
        self.do_recognition = False

        ### Can run this part once, then comment out.
        ### Will keep loading the 'mat' file.
        ###
        #cv2.imwrite('/home/hcrws1/Documents/Toolbox/sds_eccv2014/image_to_recognize.jpg',self.cv_image)
        #os.system(r"""cd /home/hcrws1/Documents/Toolbox/sds_eccv2014 && rm -rf cachedir/* && matlab -nodisplay -nosplash -nodesktop -r "run('startup_sds.m');run('demo_apc.m');exit;" """)
        ###

        rr = sio.loadmat('/home/hcr-ws/Documents/Toolbox/sds_eccv2014/recognition_results.mat')

        if self.cam_num == 0:
            self.mask_3d = rr['mask']
        elif self.cam_num in [1, 2]:
            self.mask_2d = rr['mask']
        self.categories = rr['det2cat'][0]
        self.scores = rr['det2scores'][0]
        self.centroids_x = rr['cen_x'][0]
        self.centroids_y = rr['cen_y'][0]
        self.angles_ma = rr['ang_ma'][0]
        self.angles_mi = rr['ang_mi'][0]

    	if self.cam_num == 0:
            print "publishing mask"
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(self.mask_3d, "mono8"))
    	elif self.cam_num in [1, 2]:
            print "publishing hand camera"
            self.pub_mask.publish(self.bridge.cv2_to_imgmsg(self.mask_2d, "mono8"))
        self.pub_cloud.publish(self.cloud)
        time.sleep(2);

        self.pub_rec_info.publish(MatInfo(int(self.count),
                                          self.cam_num == 0,
                                          len(self.scores),
                                          self.categories,
                                          self.scores,
                                          self.centroids_x,
                                          self.centroids_y,
                                          self.angles_ma,
                                          self.angles_mi))
        print "Recognize Status: Done"
    def acknowledge(self, req):
        self.cam_num = req.camera_num
        if self.cam_num in [0, 1, 2]:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(
                                    self.cameras[self.cam_num], "bgr8"
                                )
                self.count += 1
                self.do_recognition = True
            except CvBridgeError, e:
                print e
        else:
            print "not a valid camera number"
        return self.count

def main(args):
  r = Recognizer()
  print "initializing"
  rospy.init_node('recognizeObjects_server')
  s = rospy.Service('recognizeObjects', recognizeObjects, r.acknowledge)
  print "Ready to recognize"
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
