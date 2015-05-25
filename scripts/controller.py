#! /usr/bin/env python

import roslib
import rospy
import actionlib

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point

from apc.msg import MoveToAction, MoveToGoal


class Controller:
   def __init__(self):
	TIMEOUT = 20
      self.moveClient = actionlib.SimpleActionClient('MoveTo', MoveToAction)
      self.moveClient.wait_for_server()

   def move_to_bin(self, binName, arm):
      # Moving to a shelf location
      goal = MoveToGoal()
      goal.moveAction = MoveToGoal.MOVE_TO_SHELF
      goal.arm = arm
      goal.shelf.data = binName
      self.moveClient.send_goal(goal)
      self.moveClient.wait_for_result(rospy.Duration.from_sec(TIMEOUT))

   def move_to_pick(self, binName, arm, pose):
      # Moving to a shelf location
      goal = MoveToGoal()
      goal.moveAction = MoveToGoal.MOVE_TO_PICK
      goal.shelf.data = binName
      goal.arm = arm
      goal.movePose = pose
# if you only care about point (not orientation) the following code will keep the end-effector horizontal
# goal.movePose.position = pose.position
# goal.orientation.w = 0.7071
# goal.orientation.y = 0.7071
      self.moveClient.send_goal(goal)
      self.moveClient.wait_for_result(rospy.Duration.from_sec(TIMEOUT))

# for the point in this function, only the x and z values matter
# the robot arm will attempt to move along the y
   def move_along_line(self, binName, arm, point):
      # Moving to a shelf location
      goal = MoveToGoal()
      goal.moveAction = MoveToGoal.MOVE_ALONG_LINE
      goal.shelf.data = binName
      goal.arm = arm
      goal.movePose.position = point
      self.moveClient.send_goal(goal)
      self.moveClient.wait_for_result(rospy.Duration.from_sec(TIMEOUT))

if __name__ == '__main__':
   rospy.init_node('controller')
   controller = Controller()

# example
# I'm not sure about this part (i.e. declaring the point)
# point = Point()
# point.x = 0.5
# point.z = 1.5
# move_along_line('A', MoveToGoal.LEFT_ARM, point)