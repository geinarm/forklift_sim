#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from forklift_control.msg import FollowPathAction
from forklift_control.msg import FollowPathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header

def trajectory_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('follow_path_server', FollowPathAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.
	path = []
	path.append(Pose(Point(0.5, 0, 0), Quaternion(0,0,0,1)))
	path.append(Pose(Point(1.0, 0, 0), Quaternion(0,0,0,1)))
	path.append(Pose(Point(1.5, 0.5, 0), Quaternion(0,0,0,1)))
	path.append(Pose(Point(3.0, 2, 0), Quaternion(0,0,0,1)))
	path.append(Pose(Point(5.0, 5.0, 0), Quaternion(0,0,0,1)))
	goal = FollowPathGoal(poses=path)

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	print('waiting')
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('pubTrajectory')
		result = trajectory_client()
		print("Result:", result)
	except rospy.ROSInterruptException:
		print("program interrupted before completion")