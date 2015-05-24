#!/usr/bin/env python

# import sys
import rospy
import cv2
from apc.srv import *

def test_rec(camera):
    print "waiting err"
    # rospy.wait_for_service('recognizer_server')
    rospy.wait_for_service('recognizeObjects')
    print "done"
    try:
        recognizer = rospy.ServiceProxy('recognizeObjects', recognizeObjects)
        # print count_chars
        resp1 = recognizer(camera)
        print 'ere'
        # cv2.imshow()
        print resp1
        return "done"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "wooo"

if __name__ == "__main__":
    if len(sys.argv) == 2:
        c = int(sys.argv[1])
    else:
        c = 0
    print "attempting to recognize objects"
    print test_rec(c)
    # print "There are " + str(countChars_client(s)) + " characters in '" + s
    # "' "
