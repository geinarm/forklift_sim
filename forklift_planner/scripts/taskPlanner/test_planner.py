
from planner import *


def testStateContains():

	r1 = Robot('R1')
	p1 = Pallet('P1')
	s1 = Stack('S1')

	s1 = State()
	s2 = State()
	assert(s1.contains(s2))

	s1.set(Empty(s1))
	s1.set(Holding(r1, p1))
	s2.set(Empty(s1))
	assert(s1.contains(s2))

	s2.set(On(p1, s1))
	assert(not s1.contains(s2))


testStateContains()