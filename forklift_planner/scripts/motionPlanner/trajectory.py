
class Trajectory(object):
	def __init__(self, start, end):
		self.start = start
		self.end = end


class StraightLine(Trajectory):
	def __init__(self, start, end):
		super(StraightLine, self).__init__(start, end)


class Curve(Trajectory):
	def __init__(self, points):
		start = points[0]
		end = points[len(points)-1]
		super(Curve, self).__init__(start, end)

		self.points = points