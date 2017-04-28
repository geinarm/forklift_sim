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

from forklift_control.msg import *

from forkliftPlanner import ForkliftPlanner, WorkspacePose, ForkliftAction
from taskPlanner.predicates import *
from taskPlanner.objects import *
from taskPlanner.domain import *
from taskPlanner.taskStateParser import TaskStateParser
from motionPlanner.workspace import WorkSpace
from motionPlanner.robot import Forklift
from motionPlanner.RRT import RRT

UPDATE_RATE = 5
NODE_NAME = 'forklift_node'
ROBOT_NAME = 'forklift'

TOPIC_NAV_GOAL = 'move_base_simple/goal'
TOPIC_TASK_GOAL = 'forklift/planner/goal'

MAP_FRAME_NAME = 'map'
PLANNER_TF_PREFIX = 'forklift/planner/'


class ForkliftNode(object):
	def __init__(self):
		self.planner = ForkliftPlanner()


	def run(self):
		rospy.init_node(NODE_NAME, anonymous=True)
		self.pub_marker = rospy.Publisher('/forklift/planner/marker', Marker, queue_size=10)
		
		self.tf_broadcaster = tf.TransformBroadcaster()

		rospy.Subscriber(TOPIC_NAV_GOAL, PoseStamped, self.nav_goal_callback)
		rospy.Subscriber(TOPIC_TASK_GOAL, String, self.task_goal_callback)

		rospy.wait_for_service('gazebo/get_model_state')
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)


		rate = rospy.Rate(UPDATE_RATE)
		while not rospy.is_shutdown():
			self.updateWorldModel()
			rate.sleep()


	def navigateToPose(self, goalPoses):
		robotPose = self.planner.domain.getObject(ROBOT_NAME).pose

		ws_path = self.planner.findPath(robotPose, goalPoses)
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


	def pickupPallet(self, palletName):
		rospy.loginfo('Pickup %s', palletName)
		#self.updateWorldModel()

		client = actionlib.SimpleActionClient('pickup_pallet_server', PickupPalletAction)
		client.wait_for_server()

		actionGoal = PickupPalletGoal(palletName=String(palletName))
		client.send_goal(actionGoal)
		client.wait_for_result()
		finalState = client.get_state()

		if(finalState == GoalStatus.SUCCEEDED):
			rospy.loginfo('Success')
		else:
			rospy.loginfo('Failed %s', finalState)


	def putPallet(self, pose):
		rospy.loginfo('Put pallet down')
		#self.updateWorldModel()

		client = actionlib.SimpleActionClient('put_pallet_server', PutPalletAction)
		client.wait_for_server()

		actionGoal = PutPalletGoal(targetPose=pose)
		client.send_goal(actionGoal)
		client.wait_for_result()
		finalState = client.get_state()

		if(finalState == GoalStatus.SUCCEEDED):
			rospy.loginfo('Success')
		else:
			rospy.loginfo('Failed %s', finalState)				


	def achieveGoal(self, goalState):
		#self.updateWorldModel()

		rospy.loginfo('Planning')
		rospy.loginfo('Goal: %s', goalState)
		rospy.loginfo('Current state: %s', self.planner.currentState)

		actions = self.planner.plan(goalState)

		if (actions):
			rospy.loginfo('Found task plan with %i steps', len(actions))

			for action in actions:
				if action.type == ForkliftAction.ACTION_MOVE:
					self.navigateToPose(action.targetPoses)
				elif action.type == ForkliftAction.ACTION_TAKE:
					self.pickupPallet(action.palletName)
				elif action.type == ForkliftAction.ACTION_PUT:
					target_pose = self.workspacePoseToPose(action.targetPose)
					self.putPallet(target_pose)
				else:
					raise Exception('Unknown action type: %s', action.type)

				## Apply action to planner state
				self.planner.applyAction(action.task_action)

		else:
			rospy.loginfo('Failed to find a task plan')


	def updateWorldModel(self):
		objects = self.planner.getObjectList()
		id_counter = 0
		for obj in objects:
			if not obj.static:
				try:
					modelState = self.modelStateService(model_name=obj.name)
					pos = modelState.pose.position
					q = modelState.pose.orientation
					euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

					#self.addCubeMarker(modelState.pose, id_counter)
					
					self.planner.updateObjectPose(obj.name, pos.x, pos.y, euler[2])
				except rospy.ServiceException, e:
					print("Service call failed: %s", e)

			## Update marker
			ws_pose = self.planner.getObjectPose(obj.name)
			pose = self.workspacePoseToPose(ws_pose)
			if isinstance(obj, Pallet):
				marker = self.getPalletMarker(pose)
			elif isinstance(obj, Robot):
				marker = self.getForkliftMarker(pose)
			elif isinstance(obj, Stack):
				marker = self.getStackMarker(pose)

			marker.id = id_counter
			id_counter += 1
			self.pub_marker.publish(marker)
			label = self.getLabelMarker(pose.position, obj.name)
			label.id = id_counter
			id_counter += 1
			self.pub_marker.publish(label)

			##Brodcast transform
			world_pos = modelState.pose.position
			world_rot = modelState.pose.orientation
			self.tf_broadcaster.sendTransform(
								(world_pos.x, world_pos.y, world_pos.z),
								(world_rot.x, world_rot.y, world_rot.z, world_rot.w),
								rospy.Time.now(),
								PLANNER_TF_PREFIX+obj.name,
								MAP_FRAME_NAME)

	def nav_goal_callback(self, goal):
		ws_goal = self.poseToWorkspacePose(goal.pose.position, goal.pose.orientation)

		self.navigateToPose(ws_goal)


	def task_goal_callback(self, goal):
		rospy.loginfo('Goal reseved %s', goal.data)
		goalStr = goal.data

		parser = TaskStateParser(self.planner.domain)
		goalState = parser.parse_string(goalStr)

		self.achieveGoal(goalState);


	def poseToWorkspacePose(self, pos, q):
		euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		theta = euler[2]

		return WorkspacePose(pos.x, pos.y, theta)

	def workspacePoseToPose(self, ws_pose):
		q = tf.transformations.quaternion_from_euler(0,0, ws_pose.theta)
		pose = Pose(Vector3(ws_pose.x, ws_pose.y, 0), Quaternion(q[0], q[1], q[2], q[3]))

		return pose


	def getPalletMarker(self, pose):
		m = Marker();
		m.header.frame_id = "map"
		m.type = Marker.CUBE
		m.action = 0
		m.pose = pose
		m.scale = Vector3(1, 1.5, 0.2)
		m.color = ColorRGBA(1, 0, 0, 1)

		return m

	def getStackMarker(self, pose):
		m = Marker();
		m.header.frame_id = "map"
		m.type = Marker.LINE_STRIP
		m.action = 0
		m.pose = pose
		m.scale = Vector3(0.1, 0, 0)
		m.color = ColorRGBA(1, 1, 0, 1)
		m.points = []
		m.points.append(Point(0.75, 0.75, 0))
		m.points.append(Point(0.75, -0.75, 0))
		m.points.append(Point(-0.75, -0.75, 0))
		m.points.append(Point(-0.75, 0.75, 0))
		m.points.append(Point(0.75, 0.75, 0))

		return m

	def getForkliftMarker(self, pose):
		m = Marker();
		m.header.frame_id = "map"
		m.type = Marker.LINE_STRIP
		m.action = 0
		m.pose = pose
		m.scale = Vector3(0.1, 0, 0)
		m.color = ColorRGBA(0, 0, 1, 1)
		m.points = []
		m.points.append(Point(1.8, 0.6,0))
		m.points.append(Point(1.8, -0.6, 0))
		m.points.append(Point(-2.2, -0.6, 0))
		m.points.append(Point(-2.2, 0.6, 0))
		m.points.append(Point(1.8, 0.6,0))

		return m

	def getLabelMarker(self, pos, text):
		m = Marker();
		m.header.frame_id = "map"
		m.type = Marker.TEXT_VIEW_FACING
		m.action = 0
		m.pose = Pose(Vector3(pos.x,pos.y,1), Quaternion(0,0,0,1))
		m.scale = Vector3(1, 1, 0.3)
		m.color = ColorRGBA(0.5, 0.5, 0.5, 1)
		m.text = text

		return m


if __name__ == '__main__':
	try:
		node = ForkliftNode()
		node.run()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass