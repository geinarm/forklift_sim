import math
import numpy as np
import random

from workspace import WorkSpace
from collider import *

class Node:

	def __init__(self, point, parent, tj=None):
		self.parent = parent
		self.point = point
		self.trajectory = tj

	def getPlan(self):
		plan = []
		n = self
		while(n.parent != None):
			plan.append(n)
			n = n.parent

		plan.reverse()
		return plan


class RRT:

	def __init__(self, ws, eps, goalR, limits):
		self.ws = ws
		self.epsilon = eps
		self.goalRadius = goalR
		self.limits = limits
		self.dimentions = len(limits)


	def closestNode(self, point, robot):
		minDist = 999999
		closest = None
		for node in self.nodes:
			dist = robot.getDistance(node.point, point)
			if dist < minDist:
				closest = node
				minDist = dist

		return closest


	def inGoalRegion(self, node, robot):
		for goal in self.goals:
			dist = np.linalg.norm(node.point[0:2]-goal[0:2])
			thetaErr = abs(node.point[2]-goal[2]) % (2*math.pi)
			err = dist+thetaErr
			#dist = robot.getDistance(node.point, self.goal)
			if (dist < self.goalRadius) and (thetaErr < 0.1):
				return True

		return False


	def findPath(self, start, goals, robot, maxNodes=1000, maxSamples=1000):
		self.nodes = []
		self.start = start
		self.goals = goals
		self.solution = None

		self.nodes.append(Node(start, None))
		done = False

		samples = 0
		ng = 0
		while len(self.nodes) < maxNodes and samples < maxSamples:
			numNodes = len(self.nodes)
			pg = (float(numNodes)/maxNodes) * 0.1
			useGoal = False
			##Pick random point
			if random.random() > (1.0-pg):
				ng += 1
				useGoal = True
				point = random.choice(self.goals)
			else:
				values = []
				for i in range(self.dimentions):
					val = random.uniform(self.limits[i][0], self.limits[i][1])
					values.append(val)
				point = np.array(values)

			samples += 1
			##Find closest node
			closest = self.closestNode(point, robot)

			##Extend node towards sample point
			z = point
			#z = robot.extend(closest.point, point, self.epsilon)
			trajectory = robot.steer(closest.point, z, self.epsilon)

			if trajectory == None:
				print('None trajectory')
				continue
			newNode = Node(trajectory.end, closest, trajectory)

			##Check for collision
			if not self.ws.pointInBounds(newNode.point):
				continue

			robot.setState(newNode.point)
			collider = robot.getCollider()
			if self.ws.inCollision(collider):
				continue
			pathCollider = TrajectoryCollider(trajectory, 1.0)
			if self.ws.inCollision(pathCollider):
				continue

			##Add new node
			self.nodes.append(newNode)

			##Check goal
			if self.inGoalRegion(newNode, robot):
				self.solution = newNode
				return newNode

		print('Failed')


if __name__ == '__main__':

	ws = WorkSpace()
	ws.readObsticles('obsticles1.csv')

	rrt = RRT(ws, 30, 20)
	rrt.findPath(np.array([10, 10]), np.array([400, 400]))
