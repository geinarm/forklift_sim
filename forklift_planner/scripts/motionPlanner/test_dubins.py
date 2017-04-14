
import numpy as np
from dubins import *


def testLinePoints():
	line = Line(np.array([0,0]), np.array([0,10]))

	points = line.getPoints(ppm=1)
	print(points)
	assert(len(points) == 10)
	lastPoint = points[len(points)-1]
	print(lastPoint)
	assert(lastPoint[0:2] == [0, 10])


def testArcPoints():
	arc = Arc(np.array([0,0]), 1, np.array([1,0]), np.array([0,1]), Arc.LEFT)
	points = arc.getPoints(ppm=3)
	print(points)


	arc = Arc(np.array([1,1]), 1, np.array([2,1]), np.array([1,3]), Arc.LEFT)
	points = arc.getPoints(ppm=3)
	print(points)

#testLinePoints()
testArcPoints()