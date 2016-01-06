import viz
import vizshape
import numpy
import random
import ode
import visEnv
import physEnv

class plainObstacle():

	def __init__(self, room,height = 0.5, position=[0,0,0]):
		
		#obsHeight = 0.45
		self.height = height
		self.width= 1.24
		self.depth= 0.87

		self.position = position
		self.parentRoom = room
					
		# DepthWidthHeight
		self.node3D = visEnv.visObj(self.parentRoom,'box',[self.depth,self.width,self.height])	
		
		self.node3D.enablePhysNode()
		self.node3D.physNode.isLinked = 1;
		self.node3D.linkPhysToVis()		
		# Phys now follows viz
		
	def setColor(self,color):
		self.node3D.color(color)
						
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
		self.node3D.remove()
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
	
	print object.node3D.getPosition()
	print object.physNode.getPosition()
