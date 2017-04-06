
import copy

from objects import *
from actions import *
from predicates import *

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

	def find(self, type, *args):
		for pred in self.predicates:
			if isinstance(pred, type):
				pred.args == args
				return pred.value

		return None

	def set(self, pred):
		self.predicates.append(pred)

	def remove(self, pred):
		self.predicates.remove(pred)

	def copy(self):
		return copy.deepcopy(self)

	## True if state is subset of this state
	def contains(self, state):
		for pred in state.predicates:
			if not pred in self.predicates:
				return False
		return True

	def __str__(self):
		string = ",\n".join(['('+str(x)+')' for x in self.predicates])
		return string

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return other.predicates == self.predicates

		return False


class Domain(object):

	def __init__(self, robots=[], pallets=[], stacks=[]):
		self.robots = robots
		self.pallets = pallets
		self.stacks = stacks


	def getApplicableActions(self, state):
		actions = []
		for robot in self.robots:
			for stack in self.stacks:
				a = Align(robot, stack)
				if(a.applicable(state)):
					actions.append(a)
				
				for pallet in self.pallets:
					a = Take(robot, pallet, stack)
					if(a.applicable(state)):
						actions.append(a)

					a = Put(robot, pallet, stack)
					if(a.applicable(state)):
						actions.append(a)

		return actions



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
	start.set(Empty(s3))

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

	print(a4)
	state4 = a4.apply(state3)
	print(state4)

	print(a5)
	state5 = a5.apply(state4)
	print(state5)

	print(a6)
	state6 = a6.apply(state5)
	print(state6)

	print(a7)
	state7 = a7.apply(state6)
	print(state7)
	
	print(a8)
	state8 = a8.apply(state7)
	print(state8)

	print('Goal:')
	print(goal)


