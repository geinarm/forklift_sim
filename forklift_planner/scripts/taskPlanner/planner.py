
from Queue import PriorityQueue

from domain import *

class Node(object):
	def __init__(self, state, action, parent, cost):
		self.state = state
		self.action = action
		self.parent = parent
		self.cost = cost


class Planner(object):

	def __init__(self, domain, start, goal):
		self.start = start
		self.goal = goal
		self.domain = domain


	def findPlan(self):

		frontier = PriorityQueue()
		frontier.put((0, Node(self.start, None, None, 0)))
		count = 0

		while(not frontier.empty()):
			node = frontier.get(False)[1]
			count += 1
			if count % 100 == 0:
				print(count)

			if(node.state.contains(self.goal)):
				print('Found goal', count)
				return self.get_action_sequance(node)


			actions = self.domain.getApplicableActions(node.state)
			for action in actions:
				newState = action.apply(node.state)
				newNode = Node(newState, action, node, node.cost+1)
				frontier.put((newNode.cost, newNode))


	def get_action_sequance(self, node):
		actions = []
		n = node
		while n.parent != None:
			actions.append(n.action)
			n = n.parent

		actions.reverse()
		return actions


if __name__ == '__main__':

	robots = []
	robots.append(Robot('R1'))

	pallets = []
	pallets.append(Pallet('P1'))
	pallets.append(Pallet('P2'))

	stacks = []
	stacks.append(Stack('S1'))
	stacks.append(Stack('S2'))
	stacks.append(Stack('S3'))

	d = Domain(robots, pallets, stacks)

	start = State()
	start.set(On(pallets[0], stacks[0]))
	start.set(On(pallets[1], stacks[1]))
	start.set(Holding(robots[0], None))
	start.set(Empty(stacks[2]))

	goal = State()
	goal.set(On(pallets[0], stacks[1]))
	goal.set(Holding(robots[0], None))

	planner = Planner(d, start, goal)

	plan = planner.findPlan()

	for a in plan:
		print(a)


