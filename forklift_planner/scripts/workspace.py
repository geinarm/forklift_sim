#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3

from planningDomain.domain import *
from planner import Planner

class WorkspacePose(object):
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta


class Workspace(object):
	def __init__(self):
		self.domain = Domain()
		self.domain.robots.append(Robot('forklift'))

		self.domain.pallets.append(Pallet('pallet1'))
		self.domain.pallets.append(Pallet('pallet2'))

		self.domain.stacks.append(Stack('S1'))
		self.domain.stacks.append(Stack('S2'))
		self.domain.stacks.append(Stack('S3'))


	def run(self):
	    rospy.init_node('workspace', anonymous=True)
	    self.pub = rospy.Publisher('/forklift/planner/marks', Marker, queue_size=10)
	    rate = rospy.Rate(1) # 1hz
	    while not rospy.is_shutdown():
	    	self.getState()
	        rate.sleep()


	def getState(self):
		modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		state = State();
		idCounter = 1
		try:
			for robot in self.domain.robots:
				modelState = modelStateService(model_name=robot.name)
				robot.pose = self.poseToWorkspacePose(modelState.pose)

			for pallet in self.domain.pallets:
				modelState = modelStateService(model_name=pallet.name)
				pallet.pose = self.poseToWorkspacePose(modelState.pose)
				self.addCubeMarker(modelState.pose, idCounter)
				idCounter += 1

			self.domain.stacks[0].pose = self.domain.pallets[0].pose
			state.set(On(pallet, self.domain.stacks[0]))
			self.domain.stacks[1].pose = self.domain.pallets[1].pose
			state.set(On(pallet, self.domain.stacks[1]))
			self.domain.stacks[2].pose = WorkspacePose(10, 10, 0)

		except rospy.ServiceException, e:
			print("Service call failed: %s", e)


	def poseToWorkspacePose(self, pose):
		q = pose.orientation
		euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		theta = euler[2]
		x = pose.position.x
		y = pose.position.y

		return WorkspacePose(x, y, theta)


	def addCubeMarker(self, pose, id):
		m = Marker();
		m.header.frame_id = "map"
		m.type = 1
		m.action = 0
		m.pose = pose
		m.scale = Vector3(1, 1, 0.2)
		m.color = ColorRGBA(1, 0, 0, 1)
		m.id = id
		self.pub.publish(m)


if __name__ == '__main__':
	try:
		workspace = Workspace()
		workspace.run()
	except rospy.ROSInterruptException:
		pass