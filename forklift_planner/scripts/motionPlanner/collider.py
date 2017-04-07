
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity, geometry


class ShapeCollider(object):
	def __init__(self, shape):
		self.shape = shape


	def inCollision(self, other):
		return self.shape.intersects(other.shape)


	def getPoints(self):
		return np.array(list(self.shape.exterior.coords))


class CircleCollider(ShapeCollider):

	def __init__(self, center, radius):

		p = Point(center).buffer(radius)
		super(CircleCollider, self).__init__(p)


class RectangleCollider(ShapeCollider):

	def __init__(self, center, width, height, theta):

		left = center[0] - width/2
		right = center[0] + width/2
		bottom = center[1] - height/2
		top = center[1] + height/2

		p = geometry.box(left, bottom, right, top)
		p = affinity.rotate(p, theta, origin='center', use_radians=True)
		super(RectangleCollider, self).__init__(p)


class TrajectoryCollider(ShapeCollider):
	def __init__(self, tj, width):

		shape = LineString(tj.points)
		shape = shape.buffer(width/2, cap_style=2)

		super(TrajectoryCollider, self).__init__(shape)
		