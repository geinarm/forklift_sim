
import numpy as np
import pyglet
import math

from pyglet.gl import *

from WorkSpace import WorkSpace
from collider import *
from RRT import RRT
from robot import *
from graphics import *

CIRCLE_POINTS = 30

class WorkSpaceWindow(pyglet.window.Window):
	def __init__(self, ws, rrt, robot, width=800, height=600, scale=1):
		super(WorkSpaceWindow, self).__init__(width, height)
		self.workspace = ws
		self.graph = rrt
		self.robot = robot
		self.scale = scale
		self.set_caption('Work Space')

		self.updateGraph()


	def updateGraph(self):
		self.treeLines = []
		for node in self.graph.nodes:
			if node.trajectory != None:
				graphics = TrajectoryGraphics(node.trajectory, self.scale)
				self.treeLines.append(graphics)

		## Path
		self.pathLines = []
		success = self.graph.solution != None
		if success:
			n = self.graph.solution
			path_batch = pyglet.graphics.Batch()

			while n.parent != None:
				graphics = TrajectoryGraphics(n.trajectory, self.scale)
				self.pathLines.append(graphics)
				n = n.parent				


	def on_draw(self):
		glClearColor(240,240,240,255)
		glClear(pyglet.gl.GL_COLOR_BUFFER_BIT)

		## Obstacles
		obstacle_batch = pyglet.graphics.Batch()
		for obsticle in self.workspace.obstacles:
			points = obsticle.getPoints() * self.scale
			verts = points.flatten()
			obstacle_batch.add(len(verts)/2, GL_POLYGON, pyglet.graphics.Group(), ('v2f', verts))
			
		## Tree Lines
		line_batch = pyglet.graphics.Batch()
		node_batch = pyglet.graphics.Batch()
		for tj in self.treeLines:
			tj.draw(line_batch)

			last = len(tj.points)-1
			verts = self.makeCircle(tj.points[last][0]*self.scale, tj.points[last][1]*self.scale, 0.5)
			node_batch.add(CIRCLE_POINTS, GL_POLYGON, pyglet.graphics.Group(), ('v2f', verts))
				

		#Robot
		robot_batch = pyglet.graphics.Batch()
		self.robot.setState(self.graph.start)
		points = robot.getPoints() * self.scale
		verts = points.flatten()
		robot_batch.add(len(verts)/2, GL_LINE_LOOP, pyglet.graphics.Group(), ('v2f', verts ))

		## Path
		path_batch = pyglet.graphics.Batch()
		for tj in self.pathLines:
			tj.draw(path_batch)


		start = self.graph.start
		goal = self.graph.goal

		#if (type(self.robot) != PointRobot):
		self.robot.setState(goal)
		points = self.robot.getPoints() * self.scale
		goalVerts = points.flatten()

		self.robot.setState(start)
		points = self.robot.getPoints() * self.scale
		startVerts = points.flatten()			
		#else:
		#	goalVerts = self.makeCircle(goal[0], goal[1], self.graph.goalRadius)
		#	startVerts = self.makeCircle(start[0], start[1], 2)

		glLineWidth(1)
		##Draw goal
		glColor3f(0.1,0.5,0.1)
		pyglet.graphics.draw(len(goalVerts)/2, GL_POLYGON, ('v2f', goalVerts))
		##Draw start
		glColor3f(1,0,0)
		pyglet.graphics.draw(len(startVerts)/2, GL_POLYGON, ('v2f', startVerts))
		
		line_batch.draw()
		glColor3f(0.3,0.3,0.3)
		node_batch.draw()
		glColor3f(0,0,1)
		obstacle_batch.draw()

		glColor3f(0,1,0)
		glLineWidth(3)
		path_batch.draw()
		#glColor3f(0.5,0.5,0.5)
		#glLineWidth(1)
		#robot_batch.draw()


	def makeCircle(self, px, py, r):
		verts = []
		r = r * self.scale
		for i in range(CIRCLE_POINTS):
			angle = math.radians(float(i)/CIRCLE_POINTS * 360.0)
			x = r*math.cos(angle) + px
			y = r*math.sin(angle) + py
			verts += [x,y]
		
		return verts


if __name__ == '__main__':

	##Polygon robot
	limits = [[0,100], [0, 100], [0, 2*math.pi]]
	robot = Forklift()
	start = np.array([50, 30, 0])
	#goal = np.array([50, 60, 4.6])
	#goal = np.array([50, 60, 1.51])
	#goal = np.array([50, 60, 3.14])
	goal = np.array([49, 80, 0])
	goalRadius = 2.0
	eps = 10.0

	ws = WorkSpace(limits)
	ws.addObsticle(RectangleCollider([50,50], 40, 10, 0))

	rrt = RRT(ws, eps, goalRadius, limits)
	
	goalNode = rrt.findPath(start, goal, robot, maxNodes=100)

	window = WorkSpaceWindow(ws, rrt, robot, width=500, height=500, scale=5)

	pyglet.app.run()