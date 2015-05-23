#!/usr/bin/env python
import rospy
from control_database import StateKeeper
from apc.srv import *
from apc.msg import Recognized

database = StateKeeper()
job_numbers2d = []

def callback_recognition(datas):
	for entry in datas:
		database.addState(entry)
	launchGraspJobs()

def callback_success(data):
		if data.confidence: #==1
			if data.obj_id:
				database.successState(data)
			else:
				if bin_num%3 < 1:
					test_rec(1) #Left ARM? TODO
				else:
					test_rec(2) #Right ARM?
		else:
			database.updateConfidence(data)
		launchGraspJobs()

def listener():
    rospy.Subscriber("recognition_topic", Recognize, callback_recognition)
    rospy.Subscriber("successful_pickup", Recognize, callback_success)

def launchGraspJobs():
	x = database.getNextTarget()
	if x not in "ABCDEFGHIJKL":
		pub_move.publish(x)
	else:
		launchRecogJobs(x)

def test_rec(camera):
    print "waiting err"
    # rospy.wait_for_service('recognizer_server')
    rospy.wait_for_service('recognizeObjects')
    print "done"
    try:
        recognizer = rospy.ServiceProxy('recognizeObjects', recognizeObjects)
        # print count_chars
        resp1 = recognizer(camera)
        #print 'ere'
        # cv2.imshow()
        #print resp1
        return 1
    except rospy.ServiceException, e:
        #print "Service call failed: %s"%e
	return 0


def launchRecogJobs(recognitionType):
	if recognitionType == 0:#normal 3D Kinect
		test_rec(0)
	else:  #2D
		#move arm
		tempRecognized = Recognized()
		tempRecognized.obj_id = 0
		tempRecognized.bin_loc = recognitionType
		pub_move.publish(tempRecognized)
	

if __name__ == '__main__':
    rospy.init_node('heuristic_control', anonymous=True)
    listener()
    #pub_recognition = rospy.Publisher("/apc/recognized", Recognized)
    pub_move = rospy.Publisher("/apc/move_server", Recognized)
    rospy.spin()
