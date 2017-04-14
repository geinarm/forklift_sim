#!/usr/bin/env python

import rospy
import tf
import actionlib
import math
import numpy as np

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose

from forklift_control.msg import FollowPathAction
from forklift_control.msg import FollowPathGoal

from planningDomain.domain import *
from planner import Planner
from motionPlanner.workspace import WorkSpace
from motionPlanner.robot import Forklift
from motionPlanner.RRT import RRT

class WorkspacePose(object):
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta


class Solver(object):
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

		self.getState()

		limits = [[-20,20], [-20, 20], [0, 2*math.pi]]
		ws = WorkSpace(limits)
		rrt = RRT(ws, 1.5, 1.0, limits)

		robot = Forklift()
		start = np.array([0, 0, 0])
		goal = np.array([0, 7, math.pi])

		goalNode = rrt.findPath(start, goal, robot, maxNodes=10, maxSamples=20)

		if goalNode:
			plan = goalNode.getPlan()

			path = []
			for node in plan:
				for point in node.trajectory.points:
					path.append(Pose(Point(point[0], point[1], 0), Quaternion(0,0,0,1)))

			rospy.loginfo('Found path with %d nodes and %d points', len(plan), len(path))

			client = actionlib.SimpleActionClient('follow_path_server', FollowPathAction)
			client.wait_for_server()

			actionGoal = FollowPathGoal(poses=path)
			client.send_goal(actionGoal)
			client.wait_for_result()

		else:
			rospy.loginfo('Failed to find a path')

		#rate = rospy.Rate(1)
		#while not rospy.is_shutdown():
		#	self.getState()
		#	rate.sleep()


	def getState(self):
		modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		state = State();
		idCounter = 1
		try:
			for robot in self.domain.robots:
				modelState = modelStateService(model_name=robot.name)
				robot.pose = self.poseToWorkspacePose(modelState.pose.position, modelState.pose.orientation)
				self.addCubeMarker(modelState.pose, idCounter)
				idCounter += 1

			for pallet in self.domain.pallets:
				modelState = modelStateService(model_name=pallet.name)
				pallet.pose = self.poseToWorkspacePose(modelState.pose.position, modelState.pose.orientation)
				self.addCubeMarker(modelState.pose, idCounter)
				idCounter += 1

			self.domain.stacks[0].pose = self.domain.pallets[0].pose
			state.set(On(pallet, self.domain.stacks[0]))
			self.domain.stacks[1].pose = self.domain.pallets[1].pose
			state.set(On(pallet, self.domain.stacks[1]))
			self.domain.stacks[2].pose = WorkspacePose(10, 10, 0)

		except rospy.ServiceException, e:
			print("Service call failed: %s", e)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("TF lookup failed")
			self.tf_broadcaster.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), 'odom', 'map')



	def poseToWorkspacePose(self, pos, q):
		euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		theta = euler[2]

		return WorkspacePose(pos.x, pos.y, theta)


	def addCubeMarker(self, pose, id):
		m = Marker();
		m.header.frame_id = "map"
		m.type = 1
		m.action = 0
		m.pose = pose
		m.scale = Vector3(1, 1.5, 0.2)
		m.color = ColorRGBA(1, 0, 0, 1)
		m.id = id
		self.pub.publish(m)


if __name__ == '__main__':
	try:
		node = Solver()
		node.run()
	except rospy.ROSInterruptException:
		pass