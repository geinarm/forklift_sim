
import math
import numpy as np

from shapely.geometry import Point, Polygon
from shapely import affinity
from collider import ShapeCollider
from trajectory import *
from dubins import *

class Robot(object):

	def __init__(self, shape):
		self.position = np.array([0,0])
		self.theta = 0
		self.shape = shape
		#self.points = [[25, 0], [-10, -10], [-10, 10]]


	def setPosition(self, pos):
		self.position = pos


	def setOrientation(self, theta):
		self.theta = theta


	def getPoints(self):
		#p = Polygon(self.points)
		p = self.shape
		p = affinity.rotate(p, self.theta, origin=[0, 0], use_radians=True)

		coords = np.array(list(p.exterior.coords));
		coords = coords + self.position

		return coords


	def steer(self, p, q, eps):
		raise NotImplementedError()

	def extend(self, p, q, eps):
		raise NotImplementedError()		


	def getCollider(self):
		coords = self.getPoints()
		p = Polygon(coords)
		return ShapeCollider(p)


	def setState(self, state):
		self.position = np.array([state[0], state[1]])


	def getDistance(self, point):
		return np.linalg.norm(self.position-point)


class Forklift(Robot):

	def __init__(self):
		points = [[5, 0], [-2.5, -2.5], [-2.5, 2.5]]
		shape = Polygon(points)
		super(Forklift, self).__init__(shape)

	def setState(self, state):
		self.position = [state[0], state[1]]
		self.theta = state[2]


	def extend(self, p, q, eps):
		v = q[0:2]-p[0:2]
		length = np.linalg.norm(v)
		if length < eps:
			v = v/length
			z = p[0:2] + v*eps
			return np.array([z[0], z[1], q[2]])
		return q


	def steer(self, p, q, eps):
		calc = DubinsCalculator(3)
		dubins = calc.calculatePath(p, q)
		trajectory = dubins.getTrajectory(ppm=1, maxLength=eps)
		return trajectory


	def getPolarCoord(self, p1, p2):
		p = p1[0:2]
		p0 = p2[0:2] #np.array([point[0], point[1]])

		r = np.linalg.norm(p0-p)
		rv = (p0-p)/r

		v1 = np.array([math.cos(self.theta), math.sin(self.theta)])
		v2 = np.array([math.cos(p2[2]), math.sin(p2[2])])

		delta = math.acos(np.dot(v1,rv))
		phi = math.acos(np.dot(v2, rv))

		return np.array([r, delta, phi])


	def getDistance(self, p1, p2, k_delta=1, k_phi=1):

		polar = self.getPolarCoord(p1, p2)
		#print(polar)

		r = polar[0]
		phi = polar[1]
		delta = polar[2]
		ds = math.atan(-k_phi*polar[1])

		dist = math.sqrt(r**4 + k_phi**2 * phi**2) + k_delta * abs(delta-ds)
		return dist
