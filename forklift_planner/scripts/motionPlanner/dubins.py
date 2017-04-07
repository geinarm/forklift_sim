
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

		if theta > 0 and (self.direction == LEFT):
			theta = theta-(math.pi*2);
		elif theta < 0 and not left:
			theta = theta+(math.pi*2);

		return abs(self.radius * theta)


class DubinsPath(object):
	def __init__(self, arc1, arc2):
		self.arc1 = arc1
		self.arc2 = arc2

	def getTrajectory(self):



class RSR(DubinsPath):
	def __init__(self, p, q, T, pr, qr):
		super(RSR, self).__init__(p, q, T)

		self.radius = np.linalg.norm(p - pr)

		l1 = arcLength(pr, self.radius, self.start, T[0], False)
		l2 = np.linalg.norm(T[0] - T[1])
		l3 = arcLength(qr, self.radius, T[1], self.end, False)
		self.length = l1+l2+l3

		#steering = math.asin(5.0/self.radius)
		angularVelocity = 1/self.radius
		self.addControlSegment(SimpleCarControl(-angularVelocity, 1, l1))
		self.addControlSegment(SimpleCarControl(0, 1, l2))
		self.addControlSegment(SimpleCarControl(-angularVelocity, 1, l3))

	def info(self):
		print('RSR', 'Radius', self.radius)

class LSL(DubinsPath):
	def __init__(self, p, q, T, pl, ql):
		super(LSL, self).__init__(p, q, T)

		self.radius = np.linalg.norm(p - pl)

		l1 = arcLength(pl, self.radius, self.start, T[0], True)
		l2 = np.linalg.norm(T[0] - T[1])
		l3 = arcLength(ql, self.radius, T[1], self.end, True)
		self.length = l1+l2+l3

		#steering = math.asin(5.0/self.radius)
		angularVelocity = 1/self.radius
		self.addControlSegment(SimpleCarControl(angularVelocity, 1, l1))
		self.addControlSegment(SimpleCarControl(0, 1, l2))
		self.addControlSegment(SimpleCarControl(angularVelocity, 1, l3))
	
	def info(self):
		print('LSL', 'Radius', self.radius)		

class RSL(DubinsPath):
	def __init__(self, p, q, T, pr, ql):
		super(RSL, self).__init__(p, q, T)

		self.radius = np.linalg.norm(p - pr)

		l1 = arcLength(pr, self.radius, self.start, T[0], False)
		l2 = np.linalg.norm(T[0] - T[1])
		l3 = arcLength(ql, self.radius, T[1], self.end, True)
		self.length = l1+l2+l3

		#steering = math.asin(5.0/self.radius)
		angularVelocity = 1/self.radius
		self.addControlSegment(SimpleCarControl(-angularVelocity, 1, l1))
		self.addControlSegment(SimpleCarControl(0, 1, l2))
		self.addControlSegment(SimpleCarControl(angularVelocity, 1, l3))

	def info(self):
		print('RSL', 'Radius', self.radius)

class LSR(DubinsPath):
	def __init__(self, p, q, T, pl, qr):
		super(LSR, self).__init__(p, q, T)

		self.radius = np.linalg.norm(p - pl)

		l1 = arcLength(pl, self.radius, self.start, T[0], True)
		l2 = np.linalg.norm(T[0] - T[1])
		l3 = arcLength(qr, self.radius, T[1], self.end, False)
		self.length = l1+l2+l3

		#steering = math.asin(5.0/self.radius)
		angularVelocity = 1/self.radius
		self.addControlSegment(SimpleCarControl(angularVelocity, 1, l1))
		self.addControlSegment(SimpleCarControl(0, 1, l2))
		self.addControlSegment(SimpleCarControl(-angularVelocity, 1, l3))

	def info(self):
		print('LSR', 'Radius', self.radius)


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
		##RSRTrajectory
		T = self.getTangentPoints(pr, qr, 1, 1)
		rsr = RSR(start, end, T, pr, qr)
		paths.append(rsr)
		##LSLTrajectory
		T = self.getTangentPoints(pl, ql, -1, 1)
		lsl = LSL(start, end, T, pl, ql)
		paths.append(lsl)
		##RSLTrajectory
		T = self.getTangentPoints(pr, ql, 1, -1)
		rsl = RSL(start, end, T, pr, ql)
		paths.append(rsl)
		##LSRTrajectory
		T = self.getTangentPoints(pl, qr, -1, -1)
		lsr = LSR(start, end, T, pl, qr)
		paths.append(lsr)

		minLength = 9999
		shortest = None
		for dp in paths:
			if (dp.length < minLength):
				shortest = dp
				minLength = dp.length

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

		return np.array([p1, p2])

