import csv
import numpy as np

import collider
from collider import PointCollider, CircleCollider, RectangleCollider

class WorkSpace:

	def __init__(self, limits):
		self.obstacles = {}
		self.limits = limits


	def setObsticle(self, id, collider):
		self.obstacles[id] = collider

	def removeObsticle(self, id):
		if id in self.obstacles:
			del self.obstacles[id]


	def pointInCollision(self, point):
		collider = PointCollider(point)
		return self.inCollision(collider)


	def pointInBounds(self, point):
		for i in xrange(len(point)):
			value = point[i]
			minVal = self.limits[i][0]
			maxVal = self.limits[i][1]

			if value < minVal or value > maxVal:
				return False

		return True


	def inCollision(self, collider):
		for k,v in self.obstacles.items():
			if v.inCollision(collider):
				return True

		return False


	def readCircleObstacles(self, filename):
		with open(filename, 'rb') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				x = float(row[0])
				y = float(row[1])
				r = float(row[2])

				c = CircleCollider(np.array([x, y]), r)
				self.addObsticle(c)


	def readRectangleObstacles(self, filename):
		with open(filename, 'rb') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				x = float(row[0])
				y = float(row[1])
				w = float(row[2])
				h = float(row[3])
				t = float(row[4])

				c = RectangleCollider([x, y], w, h, t)
				self.addObsticle(c)

