
from predicates import *
from domain import State

class TaskStateParser(object):
	def __init__(self, domain):
		self.domain = domain


	def parse_string(self, stateStr):
		state = State()

		parts = stateStr.split(' ')
		if parts[0] == 'Aligned':
			predicate = self.parseAlignedPredicate(parts)
		elif parts[0] == 'Holding':
			predicate = self.parseHoldingPredicate(parts)
		elif parts[0] == 'On':
			predicate = self.parseOnPredicate(parts)
		elif parts[0] == 'Empty':
			predicate = self.parseEmptyPredicate(parts)
		else:
			raise Exception('Unknown predicate %s', parts[0])

		state.set(predicate)

		return state


	def parseAlignedPredicate(self, parts):
		robot_name = parts[1]
		target_name = parts[2]
		predicate = Aligned(self.domain[robot_name], self.domain[target_name])

		return predicate

	def parseHoldingPredicate(self, parts):
		robot_name = parts[1]
		target_name = parts[2]
		predicate = Holding(self.domain[robot_name], self.domain[target_name])

		return predicate

	def parseOnPredicate(self, parts):
		pallet_name = parts[1]
		loc_name = parts[2]
		predicate = On(self.domain[pallet_name], self.domain[loc_name])

		return predicate

	def parseEmptyPredicate(self, parts):
		obj_name = parts[1]
		predicate = Empty(self.domain[obj_name])

		return predicate
