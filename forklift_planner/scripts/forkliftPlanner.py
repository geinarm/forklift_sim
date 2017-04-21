
import math
import numpy as np

from planningDomain.domain import *
from planningDomain.objects import *
from planningDomain.actions import Take, Put, Align
from planningDomain.planner import Planner
from motionPlanner.workspace import WorkSpace
from motionPlanner.robot import Forklift
from motionPlanner.RRT import RRT
from motionPlanner.collider import *

EPS = 3.0
GOAL_RADIUS = 0.5
MAX_SEARCH_NODES = 500

ALIGN_DISTANCE = 2.5

class WorkspacePose(object):
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta

class ForkliftPlanner(object):

	def __init__(self):
		## Setup navigation domain
		limits = [[-20,20], [-20, 20], [-2*math.pi, 2*math.pi]]
		self.workspace = WorkSpace(limits)
		self.rrt = RRT(self.workspace, EPS, GOAL_RADIUS, limits)
		self.robot = Forklift()

		## Setup task domain
		self.domain = Domain()
		self.domain.robots.append(Robot('forklift'))

		self.domain.pallets.append(Pallet('pallet1'))
		self.domain.pallets.append(Pallet('pallet2'))

		self.domain.stacks.append(Stack('S1'))
		self.domain.stacks.append(Stack('S2'))
		self.domain.stacks.append(Stack('S3'))
		self.domain['S1'].pose = WorkspacePose(5, 5, 0)
		self.domain['S2'].pose = WorkspacePose(5, 0, 0)
		self.domain['S3'].pose = WorkspacePose(5, -5, 0)

		self.currentState = State()
		self.currentState.set(On(self.domain['pallet1'], self.domain['S1']))
		self.currentState.set(On(self.domain['pallet2'], self.domain['S2']))
		self.currentState.set(Holding(self.domain['forklift'], None))
		self.currentState.set(Empty(self.domain['S3']))


	def getObjectList(self):
		return ['forklift', 'pallet1', 'pallet2']

	def updateObjectState(self, name, x, y, theta):
		pose = WorkspacePose(x, y, theta)

		obj = self.domain.getObject(name)
		if obj:
			obj.pose = pose
			if self.currentState.check(Holding(self.domain['forklift'], obj)):
				self.workspace.removeObsticle(name)
			elif isinstance(obj, Pallet):
				self.workspace.setObsticle(name, RectangleCollider([x, y], 1.0, 1.5, theta))
				print('Add pallet collider at {0},{1}'.format(x,y))
		else:
			raise Exception('No such object')


	def plan(self, goal):
		planner = Planner(self.domain, self.currentState, goal)

		task_actions = planner.findPlan()
		forklift_actions = []

		for action in task_actions:
			if(isinstance(action, Take)):
				forklift_action = TakeAction(action.pallet.name)

			elif(isinstance(action, Put)):
				forklift_action = PutAction(action.pallet.name, action.stack.pose)

			elif(isinstance(action, Align)):
				target_poses = self.getStackAlignPoses(action.stack)
				forklift_action = MoveAction(target_poses)

			else:
				raise Exception('Unknow task action')

			forklift_actions.append(forklift_action)

		return forklift_actions


	def getStackAlignPoses(self, stack):
		pose = stack.pose

		pos = np.array([pose.x, pose.y])
		v = np.array([math.cos(pose.theta), math.sin(pose.theta)])
		p1 = pos + (v*ALIGN_DISTANCE)
		p2 = pos - (v*ALIGN_DISTANCE)

		pose1 = WorkspacePose(p1[0], p1[1], pose.theta+math.pi)
		pose2 = WorkspacePose(p2[0], p2[1], pose.theta)

		return [pose1, pose2]

	def findPath(self, pose1, pose2):
		start = np.array([pose1.x, pose1.y, pose1.theta])
		goal = np.array([pose2.x, pose2.y, pose2.theta])
		goalNode = self.rrt.findPath(start, goal, self.robot, maxNodes=MAX_SEARCH_NODES, maxSamples=MAX_SEARCH_NODES*3)

		if(goalNode):
			return goalNode.getPlan()
		else:
			return None


class ForkliftAction(object):
	ACTION_TAKE = 'TAKE'
	ACTION_PUT = 'PUT'
	ACTION_MOVE = 'MOVE'

	def __init__(self, type):
		self.type = type

class TakeAction(ForkliftAction):
	def __init__(self, palletName):
		super(TakeAction, self).__init__(ForkliftAction.ACTION_TAKE)
		self.palletName = palletName

class PutAction(ForkliftAction):
	def __init__(self, palletName, targetPose):
		super(PutAction, self).__init__(ForkliftAction.ACTION_PUT)
		self.palletName = palletName
		self.targetPose = targetPose

class MoveAction(ForkliftAction):
	def __init__(self, targetPoses):
		super(MoveAction, self).__init__(ForkliftAction.ACTION_MOVE)
		self.targetPoses = targetPoses ##A list of valid locations