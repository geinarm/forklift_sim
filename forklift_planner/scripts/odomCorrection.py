#!/usr/bin/env python

import rospy
import tf

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3, Quaternion, Pose

NODE_NAME = 'odom_correction'
PUBLISH_RATE = 10

MAP_FRAME_NAME = '/map'
ROBOT_FRAME_NAME = '/base_link'
ODOMETRY_FRAME_NAME = '/odom'
MODEL_NAME = 'forklift'

class OdomCorrection(object):
	def __init__(self):
		self.odom_x_offset = 0
		self.odom_y_offset = 0
		self.odom_theta_offset = 0

	def run(self):
		rospy.init_node(NODE_NAME, anonymous=True)
		self.tf_broadcaster = tf.TransformBroadcaster()
		self.tf_listener = tf.TransformListener()
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown():
			try:
				modelState = self.modelStateService(model_name=MODEL_NAME)

				world_pos = modelState.pose.position
				world_rot = modelState.pose.orientation
				world_euler = tf.transformations.euler_from_quaternion([world_rot.x, world_rot.y, world_rot.z, world_rot.w])

				(translation,rotation) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
				odom_pos = Vector3(translation[0], translation[1], translation[2])
				odom_euler = tf.transformations.euler_from_quaternion(rotation)

				xErr = world_pos.x-odom_pos.x
				yErr = world_pos.y-odom_pos.y
				thetaErr = world_euler[2]-odom_euler[2]

				self.odom_x_offset += xErr
				self.odom_y_offset += yErr
				self.odom_theta_offset += thetaErr

			except Exception as e:
				rospy.logwarn('Faild to correct frame:\n %s', e)

			self.tf_broadcaster.sendTransform(
								(self.odom_x_offset, self.odom_y_offset, 0),
								 tf.transformations.quaternion_from_euler(0, 0, self.odom_theta_offset),
								 rospy.Time.now(),
								 'odom',
								 'map')
			rate.sleep()


if __name__ == '__main__':
	try:
		node = OdomCorrection()
		node.run()
	except rospy.ROSInterruptException:
		pass