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
MAX_SPEED = 1.0

EPSILON = 0.1
MAX_PATH_ERROR = 1.0

class Line(object):
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2

		v = p2-p1
		v = v/np.linalg.norm(v)
		self._direction = v
		self._normal = np.array([v[1], -v[0]])

	def distance(self, point):
		v1 = self.p2 - self.p1
		v2 = point - self.p1

		vd = v2 - (np.dot(v2, v1) / np.linalg.norm(v1)**2) * v1
		return np.linalg.norm(vd)

	def normal(self):
		return self._normal

	def heading(self):
		v = self.p2-self.p1
		return math.atan2(v[1], v[0])

	def direction(self):
		return self._direction

	def length(self):
		return np.linalg.norm(self.p2-self.p1)


class FollowPathServer:
	def __init__(self):
		self.wheel_cmd_publisher = rospy.Publisher(WHEEL_CONTROL_TOPIC, Twist, queue_size=1)
		self.tf_listener = tf.TransformListener()

		self.pub_marker = rospy.Publisher('/path/marker', Marker, queue_size=10)

		self.server = actionlib.SimpleActionServer(ACTION_SERVER_NAME, FollowPathAction, self.execute, False)
		self.server.start()
		print('Server started')

	def execute(self, goal):
		self.done = False
		self.path = goal.poses
		self.pathIndex = 0
		self.updatePose()
		self.nextPathIndex()
		self.publishPathMarker(self.path, 99)

		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown() and not self.done:
			if self.server.is_preempt_requested():
				rospy.loginfo('Path Canceled')
				self.server.set_preempted()
				break

			self.updatePose()

			pos = np.array([self.pose[0], self.pose[1]])
			v1 = self.pathLine.p2 - pos

			pathDot = np.dot(self.pathLine.normal(), v1)
			endDot = np.dot(v1, self.pathLine.direction())

			endDistance = self.endLine.distance(pos)
			pathDistance = self.pathLine.distance(pos)

			if(pathDistance > MAX_PATH_ERROR):
				self.server.set_aborted()
				rospy.loginfo('Too far off path')
				break

			if (endDot < 0):
				## Aim for the next point on the path
				self.nextPathIndex()
				continue

			delta = self.getHeadingError()
			steering = delta*2.2 + (-np.sign(pathDot)*self.direction*pathDistance*1.8)
			target_turn = max(min(steering, MAX_TURN), -MAX_TURN)

			target_speed = 0.5*self.direction
			angular = (target_speed/LENGTH) * math.tan(target_turn)

			cmd = Twist()
			cmd.linear = Vector3(target_speed, 0, 0)
			cmd.angular = Vector3(0, 0, angular)
			self.wheel_cmd_publisher.publish(cmd)

			rate.sleep()

		self.clearPathMarker(99)

	def updatePose(self):
		try:
			(trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			euler = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])

			self.pose = [trans[0], trans[1], euler[2]]
		except:
			rospy.logwarn('Failed to lookup transform')

	def nextPathIndex(self):
		self.pathIndex +=1
		if (self.pathIndex >= len(self.path)):
			## End of the path
			position = Point(self.pose[0], self.pose[1], 0)
			q = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
			orientation = Quaternion(q[0], q[1], q[2], q[3])
			finalPose = Pose(position, orientation)

			self.server.set_succeeded(FollowPathResult(finalPose))
			cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)) ## Stop
			self.wheel_cmd_publisher.publish(cmd)
			self.done = True
			rospy.loginfo('Success')
			return

		targetPose = self.path[self.pathIndex]
		prevPose = self.path[self.pathIndex-1]
		p1 = np.array([prevPose.position.x, prevPose.position.y])
		p2 = np.array([targetPose.position.x, targetPose.position.y])
		self.pathLine = Line(p1, p2)
		self.endLine = Line(p2, p2+self.pathLine.normal())

		if (self.pathLine.length() < EPSILON):
			self.nextPathIndex()
			return

		headingErr = self.getHeadingError()
		if(abs(headingErr) > math.pi/2):
			self.direction = -1
		else:
			self.direction = 1

	def getHeadingError(self):
		targetHeading = self.pathLine.heading()

		#vx = self.pathLine.p2[0] - self.pose[0]
		#vy = self.pathLine.p2[1] - self.pose[1]
		#targetHeading = math.atan2(vy, vx)

		currentHeading = self.pose[2]
		delta = targetHeading - currentHeading
		if(delta > math.pi):
			delta -= 2*math.pi
		elif(delta < -math.pi):
			delta += 2*math.pi

		return delta

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
