
import math
import numpy as np

from trajectory import Curve

def arcLength(center, r, p1, p2, left):
	v1 = p1 - center
	v2 = p2 - center

	theta = math.atan2(v1[1], v1[0]) - math.atan2(v2[1], v2[0])

	if theta > 0 and left:
		theta = theta-(math.pi*2);
	elif theta < 0 and not left:
		theta = theta+(math.pi*2);

	return abs(r * theta)


class Line(object):
	def __init__(self, start, end):
		self.start = start
		self.end = end

	def length(self):
		return np.linalg.norm(self.start-self.end)

	def getPoints(self, ppm=1):
		points = []
		v = self.end-self.start
		numPoints = int(self.length()*ppm)
		if numPoints > 0:		
			theta = math.atan2(v[1], v[0])
			dt = 1.0/numPoints
			for i in xrange(numPoints+1):
				p = self.start + (v * (i*dt))
				points.append([p[0],p[1], theta%(2*math.pi)])

			return points
		else:
			return np.empty([0, 3])


class Arc(object):
	LEFT = 1
	RIGHT = 2

	def __init__(self, center, radius, start, end, dir):
		self.center = center
		self.radius = radius
		self.start = start
		self.end = end
		self.direction = dir


	def length(self):
		v1 = self.start - self.center
		v2 = self.end - self.center

		theta = math.atan2(v1[1], v1[0]) - math.atan2(v2[1], v2[0])

		left = (self.direction == Arc.LEFT)
		if (theta > 0) and left:
			theta = theta-(math.pi*2);
		elif (theta < 0) and not left:
			theta = theta+(math.pi*2);

		return abs(self.radius * theta)

	def thetaStart(self):
		v = self.start - self.center
		theta = math.atan2(v[1], v[0])
		return theta

	def thetaEnd(self):
		v = self.end - self.center
		theta = math.atan2(v[1], v[0])
		return theta

	def getPoints(self, ppm=1):
		points = []
		length = self.length()
		numPoints = int(length*ppm) +1
		dt = length/self.radius/numPoints

		start = self.thetaStart()
		for i in xrange(numPoints):
			if self.direction == Arc.LEFT:
				t = start + (i*dt)
				theta = t+math.pi/2
			else:
				t = start - (i*dt)
				theta = t-math.pi/2

			p = np.array([math.cos(t)*self.radius, math.sin(t)*self.radius])
			p = p+self.center
			points.append([p[0], p[1], theta%(2*math.pi)])

		return points


class DubinsPath(object):
	def __init__(self, arc1, arc2):
		self.arc1 = arc1
		self.arc2 = arc2
		self.line = Line(arc1.end, arc2.start)

	def getTrajectory(self, ppm=1, maxLength=None):
		p1 = self.arc1.getPoints(ppm)
		p2 = self.line.getPoints(ppm)
		p3 = self.arc2.getPoints(ppm)

		points = np.vstack((p1, p2, p3))
		if len(points) == 0:
			raise Exception('No trajectory found')
		if maxLength:
			length = self.length()
			numPoints = int((maxLength/length) * len(points))
			return Curve(points[0:numPoints])

		return Curve(points)

	def length(self):
		l1 = self.arc1.length()
		l2 = self.line.length()
		l3 = self.arc2.length()

		return l1+l2+l3


class RSR(DubinsPath):
	def __init__(self, p, q, T, pr, qr):

		radius = np.linalg.norm(p - pr)
		arc1 = Arc(pr, radius, p, T[0], Arc.RIGHT)
		arc2 = Arc(qr, radius, T[1], q, Arc.RIGHT)

		super(RSR, self).__init__(arc1, arc2)

class LSL(DubinsPath):
	def __init__(self, p, q, T, pl, ql):

		radius = np.linalg.norm(p - pl)
		arc1 = Arc(pl, radius, p, T[0], Arc.LEFT)
		arc2 = Arc(ql, radius, T[1], q, Arc.LEFT)

		super(LSL, self).__init__(arc1, arc2)

class RSL(DubinsPath):
	def __init__(self, p, q, T, pr, ql):

		radius = np.linalg.norm(p - pr)
		arc1 = Arc(pr, radius, p, T[0], Arc.RIGHT)
		arc2 = Arc(ql, radius, T[1], q, Arc.LEFT)

		super(RSL, self).__init__(arc1, arc2)

class LSR(DubinsPath):
	def __init__(self, p, q, T, pl, qr):

		radius = np.linalg.norm(p - pl)
		arc1 = Arc(pl, radius, p, T[0], Arc.LEFT)
		arc2 = Arc(qr, radius, T[1], q, Arc.RIGHT)

		super(LSR, self).__init__(arc1, arc2)


class DubinsCalculator(object):

	def __init__(self, r):
		self.radius = r


	def calculatePath(self, p, q):
		pl = self.getLeft(p)
		pr = self.getRight(p)
		ql = self.getLeft(q)
		qr = self.getRight(q)
		start = p[0:2]
		end = q[0:2]

		paths = []
		try:
			##RSRTrajectory
			T = self.getTangentPoints(pr, qr, 1, 1)
			rsr = RSR(start, end, T, pr, qr)
			paths.append(rsr)
		except:
			pass

		try:
			##LSLTrajectory
			T = self.getTangentPoints(pl, ql, -1, 1)
			lsl = LSL(start, end, T, pl, ql)
			paths.append(lsl)
		except:
			pass

		##RSLTrajectory
		try:
			T = self.getTangentPoints(pr, ql, 1, -1)
			rsl = RSL(start, end, T, pr, ql)
			paths.append(rsl)
		except:
			pass

		##LSRTrajectory
		try:
			T = self.getTangentPoints(pl, qr, -1, -1)
			lsr = LSR(start, end, T, pl, qr)
			paths.append(lsr)
		except:
			pass

		minLength = 9999
		shortest = None
		for dp in paths:
			length = dp.length()
			if (length < minLength):
				shortest = dp
				minLength = length

		return shortest


	def getLeft(self, p):
		x = p[0];
		y = p[1];
		theta = p[2];
		v = [math.cos(theta+(math.pi/2)), math.sin(theta+(math.pi/2))]

		return np.array([x+v[0]*self.radius, y+v[1]*self.radius])


	def getRight(self, p):
		x = p[0];
		y = p[1];
		theta = p[2];
		v = [math.cos(theta-(math.pi/2)), math.sin(theta-(math.pi/2))]

		return np.array([x+v[0]*self.radius, y+v[1]*self.radius])


	def getTangentPoints(self, c1, c2, sign1, sign2):
		r1 = self.radius
		r2 = self.radius
		d = np.linalg.norm(c2-c1)

		v = (c2-c1)/d

		c = (r1 - sign2*r2) /d
		h = math.sqrt(max(0, 1 - c**2))

		nx = v[0] * c - sign1 * h * v[1]
		ny = v[1] * c + sign1 * h * v[0]
		n = np.array([nx, ny])

		p1 = c1 + r1 * n
		p2 = c2 + sign2*r2 * n

		if (np.linalg.norm(c1-p2) < self.radius):
			raise Exception('unable to solve curve')
		if (np.linalg.norm(c2-p1) < self.radius):
			raise Exception('unable to solve curve')

		return np.array([p1, p2])

