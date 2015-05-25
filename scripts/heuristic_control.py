#!/usr/bin/env python
import rospy
from control_database import StateKeeper
from controller.py import Controller
from apc.srv import *
from apc.msg import Recognized, Recognition
from geometry_msgs.msg import Pose, Point, Quaternion
import copy

database = StateKeeper()
controller = Controller()

#pub_move = rospy.Publisher("/apc/move_server", Recognized, queue_size=20)

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
				if data.bin_num%3 < 1:
					test_rec(1) #Left ARM?
				else:
					test_rec(2) #Right ARM?
		else:
			database.updateConfidence(data)
		print "##"
#		launchGraspJobs()

def listener():
    rospy.Subscriber("/apc/recognition", Recognition, callback_recognition)
#    rospy.Subscriber("/apc/successful_pickup", Recognized, callback_success)

def launchGraspJobs():
	state, target = database.getNextTarget()
	print x
	if type(state)!=type("") : #go grab
		print "publishing to move: ",
		#pub_move.publish(x)
		binName = target[1] #char 'a'
		#binName = ord(binName) - ord('A')
		#if binName%3 < 1:
		#	arm = 1 #Left ARM?
		#else:
		#	arm = 2 #Right ARM?
		point = Point()
		point.x = state.centroid_x
		point.y = state.centroid_y
		point.z = state.centroid_z
		pose = Pose()
		pose.position = point
		q = Quaternion()
		q.x = 0.707
		q.y = 0
		q.z = 0.707
		q.w = 0
		pose.orientation = q
		arm2d, armMove = choose_arm(binName)
		if state.is3d:
			controller.move_to_pick(binName, armMove, pose)
		else:
			controller.move_along_line(binName, armMove, point)
			database.resetScores()
			launchRecogJobs(0)
	else: #Move to bin for picture
		launchRecogJobs(x)


def choose_arm(bin):
	if data.bin_num%3 < 1:
		arm2d = 1
		armMove = 101 #Left ARM
	else:
		arm2d = 2
		armMove = 100 #Right ARM
	return arm2d, arm


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
	if recognitionType == 0: #normal 3D Kinect
		test_rec(0)
	else:  #2D
		#move arm
		print "2D"
		#tempRecognized = Recognized()
		#tempRecognized.job_number = 0
		#tempRecognized.obj_id = 0 #!
		#tempRecognized.bin_loc = ord(recognitionType)
		#tempRecognized.confidence = -0.756
#		tempRecognized.centroid_x = 0
#		tempRecognized.centroid_y = 0
	#	tempRecognized.centroid_z = 0
	#	tempRecognized.angle_ma = -1
	#	tempRecognized.angle_mi = -1
	#	tempRecognized.is3d = False

#		pub_move.publish(tempRecognized)


		controller.move_to_bin(recognitionType, armMove)
		test_rec(arm2d)

		print "sent: ", tempRecognized


if __name__ == '__main__':
	rospy.init_node('heuristic_control', anonymous=True)
	listener()
	#pub_recognition = rospy.Publisher("/apc/recognized", Recognized)
	#pub_move = rospy.Publisher("/apc/move_server", Recognized, queue_size=20)
#	launchRecogJobs('A')
#	launchGraspJobs()
	rospy.spin()
