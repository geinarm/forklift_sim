
import copy

## Object Types
## -Robot
## -Pallet
## -Stack
## -Location

## Actions
## -Move
## -Align(robot, pallet)
## -Take(robot, pallet)
## -Put(robot, pallet, stack/loc)

## Predicates
## -On(pallet, stack/loc) -> [True/False]
## -Holding(robot) -> [pallet]
## -Empty(stack/loc) -> [True/False]
## -Aligned(robot, pallet) -> [True/False]

class Var(object):
	def __init__(self, name, typeName):
		self.name = name
		self.type = typeName

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
		super(Stack, self).__init__(name, 'Stack')


class State(object):
	def __init__(self):
		self.predicates = []

	def check(self, pred):
		if (pred in self.predicates):
			return True
		else:
			if(pred.value == False):
				return True ## A missing predicate is same as false predicate?

		return False

	def set(self, pred):
		self.predicates.append(pred)

	def remove(self, pred):
		self.predicates.remove(pred)

	def copy(self):
		return copy.deepcopy(self)

	def __str__(self):
		string = ",\n".join(['('+str(x)+')' for x in self.predicates])
		return string

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return other.predicates == self.predicates

		return False


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
	def __init__(self, robot, pallet, val=True):
		super(Aligned, self).__init__('Aligned', [robot, pallet], val)


class Action(object):
	def __init__(self):
		pass

class Take(Action):
	def __init__(self, robot, pallet, stack):
		self.robot = robot
		self.pallet = pallet
		self.stack = stack
		super(Take, self).__init__()

	def apply(self, state):
		if not self.applicable(state):
			raise Exception('Action is not applicable in this state')

		newState = state.copy()
		newState.set(Holding(self.robot, self.pallet))
		newState.set(Empty(self.stack))
		newState.remove(On(self.pallet, self.stack))
		newState.remove(Holding(self.robot, None))
		return newState

	def applicable(self, state):
		a = True
		a = a and state.check(Holding(self.robot, None))
		a = a and state.check(Aligned(self.robot, self.stack))
		a = a and state.check(On(self.pallet, self.stack))
		return a

	def __str__(self):
		return '{0} TAKE {1} from {2}'.format(self.robot, self.pallet, self.stack)

class Put(Action):
	def __init__(self, robot, pallet, stack):
		self.robot = robot
		self.pallet = pallet
		self.stack = stack
		super(Put, self).__init__()

	def apply(self, state):
		newState = state.copy()
		newState.remove(Empty(self.stack))
		newState.remove(Holding(self.robot, self.pallet))
		newState.set(On(self.pallet, self.stack))
		newState.set(Holding(self.robot, None))

		return newState

	def applicable(self, state):
		a = True
		a = a and state.check(Aligned(self.robot, self.stack, False))
		a = a and state.check(Holding(self.robot, self.pallet))
		a = a and state.check(Empty(self.stack))
		return a

	def __str__(self):
		return '{0} PUT {1} on {2}'.format(self.robot, self.pallet, self.stack)		

class Align(Action):
	def __init__(self, robot, stack):
		self.robot = robot
		self.stack = stack
		super(Align, self).__init__()

	def apply(self, state):
		newState = state.copy()
		newState.set(Aligned(self.robot, self.stack))
		return newState

	def applicable(self, state):
		a = True
		a = a and state.check(Aligned(self.robot, self.stack, False))
		return a

	def __str__(self):
		return '{0} ALIGN with {1}'.format(self.robot, self.stack)		


if __name__ == '__main__':

	r1 = Robot('R1')
	p1 = Pallet('P1')
	p2 = Pallet('P2')
	s1 = Stack('S1')
	s2 = Stack('S2')
	s3 = Stack('S3')

	start = State()
	start.set(On(p1, s1))
	start.set(On(p2, s2))
	start.set(Holding(r1, None))

	goal = State()
	goal.set(On(p1, s2))
	goal.set(Holding(r1, None))

	a1 = Align(r1, s2)
	a2 = Take(r1, p2, s2)
	a3 = Align(r1, s3)
	a4 = Put(r1, p2, s3)
	a5 = Align(r1, s1)
	a6 = Take(r1, p1, s1)
	a7 = Align(r1, s2)
	a8 = Put(r1, p1, s2)

	print(start)

	print(a1)
	state1 = a1.apply(start)
	print(state1)

	print(a2)
	state2 = a2.apply(state1)
	print(state2)

	print(a3)
	state3 = a3.apply(state2)
	print(state3)

	state4 = a4.apply(state3)
	print(a4)
	print(state4)

	state5 = a5.apply(state4)
	print(a5)
	print(state5)

	state6 = a6.apply(state5)
	print(a6)
	print(state6)

	state7 = a7.apply(state6)
	print(a7)
	print(state7)
	
	state8 = a8.apply(state7)
	print(a8)
	print(state8)

	print(goal)