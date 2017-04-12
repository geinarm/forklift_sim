#!/usr/bin/env python

import math
import rospy
import roslib
import actionlib
import numpy as np
import tf

from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, ColorRGBA
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point
from forklift_control.msg import FollowPathAction
from forklift_control.msg import FollowPathResult

PUBLISH_RATE = 10 # 10hz
NODE_NAME = 'follow_path_server'
ACTION_SERVER_NAME = 'follow_path_server'
WHEEL_CONTROL_TOPIC = '/forklift/front_wheel_controller/cmd_vel'

LENGTH = 1.66
MAX_TURN = math.pi/4

EPSILON = 0.2

class Line(object):
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2

		v = p2-p1
		v = v/np.linalg.norm(v)
		self.normal = np.array([v[1], -v[0]])

	def distance(self, point):
		v1 = self.p2 - self.p1
		v2 = point - self.p1

		vd = v2 - (np.dot(v2, v1) / np.linalg.norm(v1)**2) * v1
		return np.linalg.norm(vd)

	def heading(self):
		v = self.p2-self.p1
		return math.atan2(v[1], v[0])


class FollowPathServer:
	def __init__(self):
		self.wheel_cmd_publisher = rospy.Publisher(WHEEL_CONTROL_TOPIC, Twist, queue_size=1)
		self.tf_listener = tf.TransformListener()

		self.pub_marker = rospy.Publisher('/path/marker', Marker, queue_size=10)

		self.server = actionlib.SimpleActionServer(ACTION_SERVER_NAME, FollowPathAction, self.execute, False)
		self.server.start()
		print('Server started')

	def execute(self, goal):
		print('do stuff')
		path = goal.poses
		pathIndex = 1
		self.publishPathMarker(path, 99)

		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown():
			if self.server.is_preempt_requested():
				rospy.loginfo('Path Canceled')
				self.server.set_preempted()
				break

			try:
				(trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
				euler = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
			except:
				rospy.logwarn('Failed to lookup transform')
				rate.sleep()
				continue


			if pathIndex >= len(path):
				## End of the path
				finalPose = Pose(Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))
				self.server.set_succeeded(FollowPathResult(finalPose))
				cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)) ## Stop
				self.wheel_cmd_publisher.publish(cmd)
				break

			targetPose = path[pathIndex]
			prevPose = path[pathIndex-1]

			pos = np.array([trans[0], trans[1]])
			p1 = np.array([prevPose.position.x, prevPose.position.y])
			p2 = np.array([targetPose.position.x, targetPose.position.y])

			pathLine = Line(p1, p2)
			endLine = Line(p2, p2+pathLine.normal)

			endDistance = endLine.distance(pos)
			pathDistance = pathLine.distance(pos)

			if endDistance < EPSILON:
				## Aim for the next point on the path
				pathIndex += 1
				continue

			targetHeading = pathLine.heading()
			currentHeading = euler[2]
			delta = targetHeading - currentHeading

			rospy.loginfo('D1: %.2f, D2: %.2f, Delta: %.2f', endDistance, pathDistance, delta)

			cmd = Twist()
			angular = min(delta, MAX_TURN)
			cmd.angular = Vector3(0, 0, angular)
			cmd.linear = Vector3(0.5, 0, 0)

			self.wheel_cmd_publisher.publish(cmd)

			rate.sleep()

		self.clearPathMarker(99)


	def publishPathMarker(self, path, id):
		m = Marker();
		m.header.frame_id = "map"
		m.type = Marker.LINE_STRIP
		m.action = Marker.ADD
		
		m.points = []
		for pose in path:
			p = Point(pose.position.x, pose.position.y, 0.1)
			m.points.append(p)

		m.scale = Vector3(0.1, 0.1, 0.1)
		m.color = ColorRGBA(0, 1, 0, 1)
		m.id = id
		self.pub_marker.publish(m)

	def clearPathMarker(self, id):
		m = Marker();
		m.action = Marker.DELETE
		m.id = id
		self.pub_marker.publish(m)

		


if __name__ == '__main__':
	rospy.init_node(NODE_NAME)
	server = FollowPathServer()
	rospy.spin()
