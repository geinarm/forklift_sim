
import math
import numpy as np

class LineSegment(object):
	def __init__(self, p1, p2):
		self.p1 = p1
		self.p2 = p2

		v = p2-p1
		v = v/np.linalg.norm(v)
		self.normal = np.array([v[1], -v[0]])

	def distance(self, point):
		v1 = self.p2 - self.p1
		v2 = point - self.p1

		vd = v2 - (np.dot(v2, v1) / np.linalg.norm(v1)**2) * v1
		return np.linalg.norm(vd)


def testLine1():
	p1 = np.array([0, 0])
	p2 = np.array([2, 0])
	l1 = LineSegment(p1, p2)

	d = l1.distance(np.array([3, 1]))
	print(d)

def testLine2():
	p1 = np.array([0, 0])
	p2 = np.array([3, 1])
	l1 = LineSegment(p1, p2)

	d = l1.distance(np.array([-1, 3]))
	print(d)


testLine1()
testLine2()