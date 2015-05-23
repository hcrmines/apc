#!/usr/bin/env python
import rospy
from control_database import StateKeeper
from apc.srv import *
from apc.msg import Recognized, Recognition
import copy

database = StateKeeper()

pub_move = rospy.Publisher("/apc/move_server", Recognized, queue_size=20)

def callback_recognition(vectorRecognized):
	#print vectorRecognized
	print "gotCallback of size: ", len(vectorRecognized.recognitions)
	for r in vectorRecognized.recognitions:
		database.addState(r)
	launchGraspJobs()

def callback_success(data):
		if data.confidence: #==1
			print "*"
			if data.obj_id:
				database.successState(data)
			else:
				if bin_num%3 < 1:
					test_rec(1) #Left ARM? TODO
				else:
					test_rec(2) #Right ARM?
		else:
			database.updateConfidence(data)
		print "##"
#		launchGraspJobs()

def listener():
    rospy.Subscriber("/apc/recognition", Recognition, callback_recognition)
    rospy.Subscriber("/apc/successful_pickup", Recognized, callback_success)

def launchGraspJobs():
	x = database.getNextTarget()
	print x
	if type(x)!=type("") :
		print "publishing to move: ",
		print "x"
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
        print "calling camera"
        resp1 = recognizer(camera)
        #print 'ere'
        # cv2.imshow()
        #print resp1
        return 1
    except rospy.ServiceException, e:
        #print "Service call failed: %s"%e
	return 0


def launchRecogJobs(recognitionType):
	print "1"
	if recognitionType == 0:#normal 3D Kinect
		test_rec(0)
		print "0"
	else:  #2D
		#move arm
		print "2D"
		tempRecognized = Recognized()
		tempRecognized.job_number = 0
		tempRecognized.obj_id = 0 #!
		tempRecognized.bin_loc = ord(recognitionType)
		tempRecognized.confidence = -0.756
		tempRecognized.centroid_x = 0
		tempRecognized.centroid_y = 0
		tempRecognized.centroid_z = 0
		tempRecognized.angle_ma = -1
		tempRecognized.angle_mi = -1
		tempRecognized.is3d = False

		pub_move.publish(tempRecognized)

		print "sent: ", tempRecognized
	

if __name__ == '__main__':
	rospy.init_node('heuristic_control', anonymous=True)
	listener()
	#pub_recognition = rospy.Publisher("/apc/recognized", Recognized)
	#pub_move = rospy.Publisher("/apc/move_server", Recognized, queue_size=20)
	launchRecogJobs('A')
#	launchGraspJobs()
	rospy.spin()
