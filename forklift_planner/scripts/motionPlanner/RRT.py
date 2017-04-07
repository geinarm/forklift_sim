import numpy as np
import random

from WorkSpace import WorkSpace
from collider import *

class Node:

	def __init__(self, point, parent, tj=None):
		self.parent = parent
		self.point = point
		self.trajectory = tj


class RRT:

	def __init__(self, ws, eps, goalR, limits):
		self.ws = ws
		self.epsilon = eps
		self.goalRadius = goalR
		self.limits = limits
		self.dimentions = len(limits)


	def closestNode(self, point, robot, theshold=0):
		minDist = 99999
		closest = None
		for node in self.nodes:
			#dist = np.linalg.norm(point-node.point)
			dist = robot.getDistance(node.point, point)
			if dist < theshold:
				continue
			if dist < minDist:
				closest = node
				minDist = dist

		return closest


	def inGoalRegion(self, node, robot):
		dist = np.linalg.norm(node.point-self.goal)
		thetaErr = abs(node.point[2]-self.goal[2])
		err = dist+thetaErr
		#dist = robot.getDistance(node.point, self.goal)
		return (dist < self.goalRadius) and (thetaErr < 0.2)


	def findPath(self, start, goal, robot, maxNodes=1000):
		self.nodes = []
		self.start = start
		self.goal = goal
		self.solution = None

		self.nodes.append(Node(start, None))
		done = False

		#MyRand = [ [20, 80, 4], [70, 50, 5], [75, 30, 4] ]
		#ri = 0
		ng = 0
		while len(self.nodes) < maxNodes:
			numNodes = len(self.nodes)
			pg = (float(numNodes)/maxNodes) * 0.3
			useGoal = False
			##Pick random point
			if random.random() > (1.0-pg):
				ng += 1
				print('Use goal', ng)
				useGoal = True
				point = goal
			else:
				values = []
				for i in range(self.dimentions):
					val = random.uniform(self.limits[i][0], self.limits[i][1])
					values.append(val)
				point = np.array(values)

			##Find closest node
			if useGoal:
				n = self.closestNode(point, robot, 5)
			else:
				n = self.closestNode(point, robot)

			##Extend node towards sample point
			z = robot.extend(n.point, point)
			trajectory = robot.steer(n.point, z, self.epsilon)
			if trajectory == None:
				continue
			if useGoal:
				print(trajectory.end)
			newNode = Node(trajectory.end, n, trajectory)

			##Check for collision
			if not self.ws.pointInBounds(newNode.point):
				continue;

			robot.setState(newNode.point)
			collider = robot.getCollider()
			if self.ws.inCollision(collider):
				continue
			pathCollider = TrajectoryCollider(trajectory, 5.0)
			if self.ws.inCollision(pathCollider):
				continue

			##Add new node
			self.nodes.append(newNode)

			##Check goal
			if self.inGoalRegion(newNode, robot):
				print('Success')
				self.solution = newNode
				return newNode

		print('Failed')


if __name__ == '__main__':

	ws = WorkSpace()
	ws.readObsticles('obsticles1.csv')

	rrt = RRT(ws, 30, 20)
	rrt.findPath(np.array([10, 10]), np.array([400, 400]))
