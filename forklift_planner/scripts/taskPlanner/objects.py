
class Var(object):
	def __init__(self, name, typeName, static=False):
		self.name = name
		self.type = typeName
		self.static = static

	def __str__(self):
		return self.name

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return other.name == self.name

		return False		

class Robot(Var):
	def __init__(self, name):
		super(Robot, self).__init__(name, 'Robot')

class Pallet(Var):
	def __init__(self, name):
		super(Pallet, self).__init__(name, 'Pallet')

class Stack(Var):
	def __init__(self, name):
		super(Stack, self).__init__(name, 'Stack', True)
