
import math
import numpy as np

class Line(object):
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2

		v = p2-p1
		v = v/np.linalg.norm(v)
		self._direction = v
		self._normal = np.array([v[1], -v[0]])

	def distance(self, point):
		v1 = self.p2 - self.p1
		v2 = point - self.p1

		vd = v2 - (np.dot(v2, v1) / np.linalg.norm(v1)**2) * v1
		return np.linalg.norm(vd)

	def normal(self):
		return self._normal

	def heading(self):
		v = self.p2-self.p1
		return math.atan2(v[1], v[0])

	def direction(self):
		return self._direction

	def length(self):
		return np.linalg.norm(self.p2-self.p1)

