import viz
import vizshape
import numpy
import random
import ode
import visEnv
import physEnv

class obstacleObj():

	def __init__(self, room,crossBarHeight=0.5, obsPosition=[0,0,0]):
		
		obsHeight = 0.95
		obsWidth = 1.24
		obsDepth = 0.87
		radius = 0.01
		
		self.parentRoom = room
		
		self.crossBarHeight = crossBarHeight
		self.obsPosition = obsPosition
				
		self.bottomMainPipe= vizshape.addCylinder( obsWidth, radius, axis = vizshape.AXIS_X, bottom = True, top = True, color = viz.WHITE)
		
		self.rightVertPipe = vizshape.addCylinder( obsHeight, radius, axis = vizshape.AXIS_Y, bottom = True, top = True, color = viz.WHITE)
		self.leftVertPipe = vizshape.addCylinder( obsHeight, radius, axis = vizshape.AXIS_Y, bottom = True, top = True, color = viz.WHITE)

		self.rightVertPipe.setParent(self.bottomMainPipe)
		self.leftVertPipe.setParent(self.bottomMainPipe)
		
		self.rightVertPipe.setPosition([obsWidth/2,obsHeight/2,0])	
		self.leftVertPipe.setPosition([-obsWidth/2,obsHeight/2,0])	
		
		self.bottomRightPipe = vizshape.addCylinder( obsDepth, radius, axis = vizshape.AXIS_Z, bottom = True, top = True, color = viz.WHITE)
		self.bottomLeftPipe = vizshape.addCylinder( obsDepth, radius, axis = vizshape.AXIS_Z, bottom = True, top = True, color = viz.WHITE)

		self.bottomRightPipe.setParent(self.bottomMainPipe)
		self.bottomLeftPipe.setParent(self.bottomMainPipe)
		
		self.bottomRightPipe.setPosition([obsWidth/2,0,0])
		self.bottomLeftPipe.setPosition([-obsWidth/2,0,0])
		
		self.bottomMainPipe.setPosition(self.obsPosition)
		
		## Note that the crossbar pipe is a visObj, not a standard Vizard viznode
		self.crossbarPipe = vizshape.addCylinder( obsWidth, radius, axis = vizshape.AXIS_X, bottom = True, top = True, color = viz.WHITE)
		self.crossbarPipe.setParent(self.bottomMainPipe)
		self.crossbarPipe.setPosition([0, self.crossBarHeight, 0],viz.ABS_PARENT)
		
		
		## I cheat, because it is difficult to resize a viznode after making it
		## I set the collision box to be very tall, but set the position (not size) so that the top 
		## ends at the crossbar. 
		
		self.collisionBoxSize = 3.0
		
		# DWH
		self.collisionBox = visEnv.visObj(self.parentRoom,'box',[radius,obsWidth,self.collisionBoxSize])		
		self.collisionBox.enablePhysNode()
		self.collisionBox.toggleUpdatePhysWithVis()
		
		self.collisionBoxLink = viz.link(self.bottomMainPipe,self.collisionBox.visNode)
		boxYPosition = -self.collisionBoxSize/2 + self.crossBarHeight
		self.collisionBoxLink.setOffset([0,boxYPosition,0])
			
		self.collisionBox.visNode.setPosition([0, boxYPosition, 0],viz.ABS_PARENT)
		self.collisionBox.visNode.visible(viz.OFF)
		
		self.visNode = viz.addGroup()
		self.bottomMainPipe.setParent(self.visNode)
		
	def setColor(self,color):
		self.visNode.color(color)
		
	def toggleCBoxVisibility(self):
		self.collisionBox.visNode.visible(viz.TOGGLE)
		
	def setCrossbarHeight(self, crossBarHeight):
		
		boxYPosition = -self.collisionBoxSize/2 + crossBarHeight
		#self.collisionBoxLink.setPosition([0,boxYPosition,0])

		self.collisionBoxLink.setOffset([0,boxYPosition,0])
		#myObstacle.collisionBoxLink.setOffset([0,-.2,0])
		
		self.crossbarPipe.setPosition([0, crossBarHeight, 0],viz.ABS_PARENT)
		self.crossBarHeight = crossBarHeight
		
	def setPosition(self, position):
		
		self.obsPosition = position
		self.bottomMainPipe.setPosition(self.obsPosition,viz.ABS_PARENT)

	def getPosition(self):

		return self.obsPosition
		
	

	def getHeight(self):
		
		return self.obsHeight

	def setEuler(self, euler):
	
		self.bottomMainPipe.setEuler(euler)
	
	def remove(self):
		self.visNode.remove()
		pass
		
def updateObstacle():
	global myObstacle
	myObstacle.setHeight(random.uniform(.1, 1))
	myObstacle.setPosition([random.uniform(-1, 1), 0, random.uniform(-1,1)])
	
	print 'Placing Obstacle at', myObstacle.getPosition(),'with', myObstacle.getHeight(), '(m) obsHeight\n'

if __name__ == "__main__":

	#Enable full screen anti-aliasing (FSAA) to smooth edges
	viz.setMultiSample(4)
	viz.vsync(1)
	viz.go()
	
	#Increase the Field of View
	viz.MainWindow.fov(60)

	#piazza = viz.addChild('piazza.osgb')
	
	viz.MainView.setPosition([-6, 0.5, 0])
	viz.MainView.lookAt([-2,0,0])

	room = visEnv.room()
	
	myObstacle = obstacleObj(room, 0.4, [0,0,.5])
	myObstacle.setPosition([-2,0,.5])
	
	import testPhys
	ball = testPhys.physBall(room)
	
	myObstacle.collisionBox.toggleUpdatePhysWithVis()
	
	vizact.onkeydown('1',ball.launchBall1)
	vizact.onkeydown('2',ball.launchBall2)
	vizact.onkeydown('3',ball.launchBall3)
		
	#myObstacle.toggleCBoxVisibility()
	
	#myTimerAction = vizact.ontimer(1,updateObstacle) 

def queryVisAndPhys(object):
	
	print object.visNode.getPosition()
	print object.physNode.getPosition()
