
from predicates import *
from objects import *


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
		if not self.applicable(state):
			raise Exception('Action is not applicable in this state')

		newState = state.copy()
		newState.remove(Empty(self.stack))
		newState.remove(Holding(self.robot, self.pallet))
		newState.set(On(self.pallet, self.stack))
		newState.set(Holding(self.robot, None))

		return newState

	def applicable(self, state):
		a = True
		a = a and state.check(Aligned(self.robot, self.stack))
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

		aligned = newState.find(Aligned, self.robot)
		if aligned != None:
			newState.remove(Aligned(self.robot, aligned))
		newState.set(Aligned(self.robot, self.stack))
		return newState

	def applicable(self, state):
		a = True
		a = a and not state.check(Aligned(self.robot, self.stack))
		return a

	def __str__(self):
		return '{0} ALIGN with {1}'.format(self.robot, self.stack)		

