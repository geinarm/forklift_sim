
import math
import numpy as np
import reeds_shepp as rp

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
		points = [[1.8, 0.6], [1.8, -0.6], [-2.2, -0.6], [-2.2, 0.6]]
		shape = Polygon(points)
		super(Forklift, self).__init__(shape)
		
		self.turn_radius = 2.0

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
		delta = 0.2
		length = rp.path_length(p, q, self.turn_radius)
		points = rp.path_sample(p, q, self.turn_radius, delta)
		points.append(q)
		path = np.array(points)

		if length > eps:
			num_points = math.floor(eps/delta)
			path = path[0:num_points+1]

		return Curve(path)
		#calc = DubinsCalculator(3)
		#dubins = calc.calculatePath(p, q)
		#trajectory = dubins.getTrajectory(ppm=10, maxLength=eps)
		#return trajectory

	def getDistance(self, p1, p2):

		length = rp.path_length(p1, p2, self.turn_radius)
		return length
		#d = np.linalg.norm(p1[0:2] - p2[0:2])
		#thetaErr = abs(p1[2] - p2[2]) % math.pi

		#return d + (4*thetaErr)
