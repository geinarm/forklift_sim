#!/usr/bin/env python

import math
import rospy
import roslib
import actionlib
import numpy as np
import tf

from utils import Line
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetLinkState
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point

from forklift_control.msg import PutPalletAction

PUBLISH_RATE = 10 # 10hz
NODE_NAME = 'put_pallet_server'
ACTION_SERVER_NAME = 'put_pallet_server'
WHEEL_CMD_TOPIC = '/forklift/front_wheel_controller/cmd_vel'
FORK_CMD_TOPIC = '/forklift/lift_controller/cmd_vel'
LIFT_CONTROL_TOPIC = '/forklift/lift_controller/command'
LIFT_LINK_NAME = 'forklift::forklift_lift1'

MAP_FRAME = '/map'
ROBOT_FRAME = '/base_link'
MAX_DISTANCE = 3.0
ALIGN_DISTANCE = 1.4
BACKUP_DISTANCE = 2.5
BACKUP_SPEED = 0.3

LENGTH = 1.66
MAX_TURN = math.pi/4

CONTROL_STATE_ALIGN = 1
CONTROL_STATE_LIFT = 2
CONTROL_STATE_BACKUP = 3
CONTROL_STATE_DONE = 4


class PutPalletServer:
	def __init__(self):
		self.wheel_cmd_publisher = rospy.Publisher(WHEEL_CMD_TOPIC, Twist, queue_size=1)
		self.lift_cmd_publisher = rospy.Publisher(LIFT_CONTROL_TOPIC, Float64, queue_size=1)
		self.tf_listener = tf.TransformListener()

		rospy.wait_for_service('gazebo/get_model_state')
		rospy.wait_for_service('gazebo/get_link_state')
		self.modelStateService = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
		self.linkStateService = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

		self.server = actionlib.SimpleActionServer(ACTION_SERVER_NAME, PutPalletAction, self.execute, False)
		self.server.start()
		print('Server started')


	def execute(self, goal):
		rate = rospy.Rate(PUBLISH_RATE)
		self.targetPose = goal.targetPose
		self.state = CONTROL_STATE_ALIGN
		self.start_time = rospy.get_time()
		self.lookupTransfrom()
		self.findTarget()

		while not rospy.is_shutdown():
			if self.server.is_preempt_requested():
				rospy.loginfo('Pickup Canceled')
				self.server.set_preempted()
				break

			self.lookupTransfrom()

			if self.state == CONTROL_STATE_ALIGN:
				self.control_alignment()
			elif self.state == CONTROL_STATE_LIFT:
				self.control_lift()
			elif self.state == CONTROL_STATE_BACKUP:
				self.control_backup()
			elif self.state == CONTROL_STATE_DONE:
				self.server.set_succeeded()
				rospy.loginfo('Success')
				break

			rate.sleep()


	def findTarget(self):
		pos = self.targetPose.position
		self.pallet_position = np.array([pos.x, pos.y])
		q = self.targetPose.orientation
		euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.pallet_theta = euler[2]

		v_heading = np.array([math.cos(self.robot_theta), math.sin(self.robot_theta)])
		v = np.array([math.cos(self.pallet_theta), math.sin(self.pallet_theta)])
		if(np.dot(v, v_heading) > 0):
			self.target_position = self.pallet_position - (v*ALIGN_DISTANCE)
			self.target_theta = self.pallet_theta
			self.target_line = Line(self.target_position, self.target_position-v)
		else:
			self.target_position = self.pallet_position + (v*ALIGN_DISTANCE)
			self.target_theta = (self.pallet_theta + math.pi)
			self.target_line = Line(self.target_position, self.target_position+v)


	def lookupTransfrom(self):
		try:
			(trans,rot) = self.tf_listener.lookupTransform(MAP_FRAME, ROBOT_FRAME, rospy.Time(0))
			euler = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
			self.robot_position = np.array([trans[0], trans[1]])
			self.robot_theta = euler[2]

		except Exception as e:
			rospy.logwarn('Failed to lookup transform')
			print(e)
			return 0


	def control_alignment(self):
		v_pallet = self.pallet_position-self.robot_position
		v_target = self.target_position-self.robot_position
		v_heading = np.array([math.cos(self.robot_theta), math.sin(self.robot_theta)])
		distance = np.linalg.norm(v_pallet)			

		if distance < ALIGN_DISTANCE:
			#self.server.set_succeeded()
			rospy.loginfo('Aligned')
			self.state = CONTROL_STATE_LIFT
			return

		theta_target = math.atan2(v_target[1], v_target[0])

		lineDot = np.dot(self.target_line.normal(), v_pallet)
		lineDistance = self.target_line.distance(self.robot_position)
		lineErr = np.sign(lineDot)*lineDistance

		alignErr = self.target_theta - self.robot_theta
		if(alignErr > math.pi):
			alignErr -= 2*math.pi
		elif(alignErr < -math.pi):
			alignErr += 2*math.pi

		headingErr = theta_target - self.robot_theta
		if(headingErr > math.pi):
			headingErr -= 2*math.pi
		elif(headingErr < -math.pi):
			headingErr += 2*math.pi				

		if lineDistance > 0.5:
			steering = (alignErr*0.2) + (headingErr*1.5) + (lineErr*1.0)
		elif distance > 2.4:
			steering = (alignErr*2.0) + (headingErr*4.0) + (lineErr*0.5)
		else:
			steering = (alignErr*3.0) + (headingErr*0.0) + (lineErr*1.0)

		target_turn = max(min(steering, MAX_TURN), -MAX_TURN)

		rospy.loginfo('HErr: %.2f, AErr: %.2f, PDist: %.2f, Steering: %.2f, Dist: %.2f', headingErr, alignErr, lineErr, steering, distance)

		target_speed = 0.2
		angular = (target_speed/LENGTH) * math.tan(target_turn)

		cmd = Twist()
		cmd.linear = Vector3(target_speed, 0, 0)
		cmd.angular = Vector3(0, 0, angular)
		self.wheel_cmd_publisher.publish(cmd)


	def control_lift(self):

		lift_link = self.linkStateService(link_name=LIFT_LINK_NAME)
		height = lift_link.link_state.pose.position.z

		if height > 0.05:
			self.lift_cmd_publisher.publish(-0.5)
		else:
			self.lift_cmd_publisher.publish(0)
			rospy.loginfo('Lift is down')
			self.state = CONTROL_STATE_BACKUP


	def control_backup(self):
		v_pallet = self.pallet_position-self.robot_position
		distance = np.linalg.norm(v_pallet)

		if(distance < BACKUP_DISTANCE):
			cmd = Twist()
			cmd.linear = Vector3(-BACKUP_SPEED, 0, 0)
			cmd.angular = Vector3(0, 0, 0)
			self.wheel_cmd_publisher.publish(cmd)
		else:
			cmd = Twist()
			cmd.linear = Vector3(0, 0, 0)
			cmd.angular = Vector3(0, 0, 0)
			self.wheel_cmd_publisher.publish(cmd)
			self.state = CONTROL_STATE_DONE


if __name__ == '__main__':
	rospy.init_node(NODE_NAME)
	server = PutPalletServer()
	rospy.spin()
