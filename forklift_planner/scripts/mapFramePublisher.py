#!/usr/bin/env python

import rospy
import tf

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3, Quaternion, Pose

NODE_NAME = 'perfect_position'
PUBLISH_RATE = 50

MAP_FRAME_NAME = 'map'
ROBOT_FRAME_NAME = 'base_link'
MODEL_NAME = 'forklift'

class MapFramePublisher(object):
	def __init__(self):
		pass

	def run(self):
		rospy.init_node(NODE_NAME, anonymous=True)
		self.tf_broadcaster = tf.TransformBroadcaster()
		self.tf_listener = tf.TransformListener()

		rospy.wait_for_service('gazebo/get_model_state')
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown():
			try:
				modelState = self.modelStateService(model_name=MODEL_NAME)

				world_pos = modelState.pose.position
				world_rot = modelState.pose.orientation

				self.tf_broadcaster.sendTransform(
									(world_pos.x, world_pos.y, world_pos.z),
									(world_rot.x, world_rot.y, world_rot.z, world_rot.w),
									rospy.Time.now(),
									ROBOT_FRAME_NAME,
									MAP_FRAME_NAME)

			except Exception as e:
				rospy.logwarn('Faild to get model state:\n %s', e)

			rate.sleep()


if __name__ == '__main__':
	try:
		node = MapFramePublisher()
		node.run()
	except rospy.ROSInterruptException:
		pass