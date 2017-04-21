#!/usr/bin/env python

import math
import rospy
import roslib
import actionlib
import numpy as np
import tf

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point

from forklift_control.msg import PickupPalletAction

PUBLISH_RATE = 10 # 10hz
NODE_NAME = 'pickup_pallet_server'
ACTION_SERVER_NAME = 'pickup_pallet_server'
WHEEL_CMD_TOPIC = '/forklift/front_wheel_controller/cmd_vel'
FORK_CMD_TOPIC = '/forklift/lift_controller/cmd_vel'

ROBOT_FRAME = '/base_link'
MAX_DISTANCE = 2.0
LENGTH = 1.66
MAX_TURN = math.pi/4
MAX_SPEED = 1.0


class PickupPalletServer:
	def __init__(self):
		self.wheel_cmd_publisher = rospy.Publisher(WHEEL_CMD_TOPIC, Twist, queue_size=1)

		rospy.wait_for_service('gazebo/get_model_state')
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		self.server = actionlib.SimpleActionServer(ACTION_SERVER_NAME, PickupPalletAction, self.execute, False)
		self.server.start()
		print('Server started')


	def execute(self, goal):
		rate = rospy.Rate(PUBLISH_RATE)
		self.targetPallet = goal.palletName

		self.getOffset()

		pos = np.array([pose.x, pose.y])
		v = np.array([math.cos(self.pallet_theta), math.sin(self.pallet_theta)])
		if(abs(self.pallet_theta-self.robot_theta) < math.pi/2)
		p1 = pos + (v*ALIGN_DISTANCE)
		p2 = pos - (v*ALIGN_DISTANCE)		

		while not rospy.is_shutdown() and not self.done:
			if self.server.is_preempt_requested():
				rospy.loginfo('Pickup Canceled')
				self.server.set_preempted()
				break

			offset = self.getOffset()
			v = offset[0]
			distance = np.linalg.norm(v)
			

			if(distance > MAX_DISTANCE):
				self.server.set_aborted()
				rospy.loginfo('Pallet is out or range')
				break

			target_speed = 0.2
			angular = (target_speed/LENGTH) * math.tan(target_turn)

			cmd = Twist()
			cmd.linear = Vector3(target_speed, 0, 0)
			cmd.angular = Vector3(0, 0, angular)
			self.wheel_cmd_publisher.publish(cmd)

			rate.sleep()


	def getOffset(self):
		try:
			(trans,rot) = self.tf_listener.lookupTransform('/map', ROBOT_FRAME, rospy.Time(0))
			euler = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
			self.robot_position = np.array([trans[0], trans[1]])
			self.robot_theta = euler[2]

			modelState = self.modelStateService(model_name=self.targetPallet)
			self.pallet_position = np.array([modelState.position.x, modelState.position.y])
			q = modelState.pose.orientation
			euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			self.pallet_theta = euler[2]

			return (pallet_position-robot_position, pallet_theta-robot_theta)

		except:
			rospy.logwarn('Failed to lookup transform')


if __name__ == '__main__':
	rospy.init_node(NODE_NAME)
	server = PickupPalletServer()
	rospy.spin()
