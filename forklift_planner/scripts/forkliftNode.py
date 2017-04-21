#!/usr/bin/env python

import rospy
import tf
import actionlib
import math
import numpy as np

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, PoseStamped

from forklift_control.msg import FollowPathAction
from forklift_control.msg import FollowPathGoal

from forkliftPlanner import ForkliftPlanner, WorkspacePose, ForkliftAction
from planningDomain.predicates import *
from planningDomain.domain import *
from motionPlanner.workspace import WorkSpace
from motionPlanner.robot import Forklift
from motionPlanner.RRT import RRT

NODE_NAME = 'forklift_node'
ROBOT_NAME = 'forklift'

TOPIC_NAV_GOAL = 'move_base_simple/goal'
TOPIC_TASK_GOAL = 'forklift/planner/goal'


class ForkliftNode(object):
	def __init__(self):
		self.planner = ForkliftPlanner()


	def run(self):
		rospy.init_node(NODE_NAME, anonymous=True)
		self.pub = rospy.Publisher('/forklift/planner/marks', Marker, queue_size=10)

		rospy.Subscriber(TOPIC_NAV_GOAL, PoseStamped, self.nav_goal_callback)
		rospy.Subscriber(TOPIC_TASK_GOAL, String, self.task_goal_callback)

		rospy.wait_for_service('gazebo/get_model_state')
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		self.updateWorldModel()


	def navigateToPose(self, goalPose):
		self.updateWorldModel()

		robotPose = self.planner.domain.getObject(ROBOT_NAME).pose

		ws_path = self.planner.findPath(robotPose, goalPose)
		if(ws_path):
			path = []
			for node in ws_path:
				for point in node.trajectory.points:
					path.append(Pose(Point(point[0], point[1], 0), Quaternion(0,0,0,1)))

			rospy.loginfo('Found path with %d nodes and %d points', len(ws_path), len(path))

			client = actionlib.SimpleActionClient('follow_path_server', FollowPathAction)
			client.wait_for_server()

			actionGoal = FollowPathGoal(poses=path)
			client.send_goal(actionGoal)
			client.wait_for_result()
			finalState = client.get_state()

			if(finalState == GoalStatus.SUCCEEDED):
				rospy.loginfo('Success')
			else:
				rospy.loginfo('Failed %s', finalState)

		else:
			rospy.loginfo('Failed to find a path')

	def achieveGoal(self, goalState):
		self.updateWorldModel()

		rospy.loginfo('Planning')
		rospy.loginfo('Goal: %s', goalState)
		rospy.loginfo('Current state: %s', self.planner.currentState)

		actions = self.planner.plan(goalState)

		if (actions):
			rospy.loginfo('Found task plan')

			for action in actions:
				if action.type == ForkliftAction.ACTION_MOVE:
					self.navigateToPose(action.targetPoses[0])
				else:
					raise Exception('Unknown action type: %s', action.type)

		else:
			rospy.loginfo('Failed to find a task plan')


	def updateWorldModel(self):
		objects = self.planner.getObjectList()
		id_counter = 0
		for objName in objects:
			try:
				modelState = self.modelStateService(model_name=objName)
				pos = modelState.pose.position
				q = modelState.pose.orientation
				euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

				self.addCubeMarker(modelState.pose, id_counter)
				id_counter += 1

				self.planner.updateObjectState(objName, pos.x, pos.y, euler[2])
			except rospy.ServiceException, e:
				print("Service call failed: %s", e)


	def nav_goal_callback(self, goal):
		ws_goal = self.poseToWorkspacePose(goal.pose.position, goal.pose.orientation)

		self.navigateToPose(ws_goal)


	def task_goal_callback(self, goal):
		rospy.loginfo('Goal reseved %s', goal.data)
		d = self.planner.domain
		goalState = State()
		goalStr = goal.data

		parts = goalStr.split(' ')
		if parts[0] == 'Aligned':
			robot_name = parts[1]
			target_name = parts[2]
			predicate = Aligned(d[robot_name], d[target_name])
			goalState.set(predicate)
		else:
			raise Exception('Unknown predicate %s', parts[0])

		self.achieveGoal(goalState);


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
		node = ForkliftNode()
		node.run()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass