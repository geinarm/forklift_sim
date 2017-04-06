
class Predicate(object):
	def __init__(self, name, args, value):
		self.name = name
		self.args = args
		self.value = value

	def eval(self):
		return self.value

	def __str__(self):
		argStr = ",".join([str(x) for x in self.args])
		return '{0}({1}) = {2}'.format(self.name, argStr, self.value)

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return self.name == other.name and\
			self.value == other.value and\
			self.args == other.args
		return False


class On(Predicate):
	def __init__(self, pallet, loc, val=True):
		super(On, self).__init__('On', [pallet, loc], val)

class Holding(Predicate):
	def __init__(self, robot, val):
		super(Holding, self).__init__('Holding', [robot], val)

class Empty(Predicate):
	def __init__(self, loc, val=True):
		super(Empty, self).__init__('Empty', [loc], val)

class Aligned(Predicate):
	def __init__(self, robot, val):
		super(Aligned, self).__init__('Aligned', [robot], val)
