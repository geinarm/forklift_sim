import csv
import numpy as np

import collider
from collider import CircleCollider, RectangleCollider

class WorkSpace:

	def __init__(self, limits):
		self.obstacles = []
		self.limits = limits


	def addObsticle(self, collider):
		self.obstacles.append(collider)


	def pointInCollision(self, point):
		for c in self.obstacles:
			if c.type == collider.COLLIDER_TYPE_CIRCLE:
				dist = np.linalg.norm(c.center - point)
				if dist < c.radius:
					return True

			elif c.type == collider.COLLIDER_TYPE_POLYGON:
				if c.containsPoint(point):
					return True

		return False


	def pointInBounds(self, point):
		for i in xrange(len(point)):
			value = point[i]
			minVal = self.limits[i][0]
			maxVal = self.limits[i][1]

			if value < minVal or value > maxVal:
				return False

		return True


	def inCollision(self, collider):
		for c in self.obstacles:
			if c.inCollision(collider):
				return True

		return False


	def polygonInCollision(self, collider):
		for c in self.obstacles:
			if c.inCollision(collider):
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


	def readPolygonObstacles(self, filename):
		with open(filename, 'rb') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				x = float(row[0])
				y = float(row[1])
				r = float(row[2])

				points = [[-r, -r], [r, -r], [r, r], [-r, r]]
				center = [x, y]

				c = PolygonCollider(np.array(center), np.array(points))
				c.setTheta(0.5)
				self.addObsticle(c)

