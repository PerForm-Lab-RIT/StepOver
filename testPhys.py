        
from visEnv import *
        
import vizact
useConfig = True


class physBall():
	def __init__(self,room,ballInitialPos = [0, 2, 0]):
			
		self.ball = visObj(room,'sphere',.04,ballInitialPos,[0,1,1])
		self.ball.visNode.alpha(1)
		self.ball.enablePhysNode()
		#self.ball.setVelocity(ballInitialVel)	
		self.ball.toggleUpdateWithPhys()
		self.ball.physNode.enableMovement()
		
	def launchBall1(self):
		
		launchVel_XYZ = [ 0,0,-3 ]
		self.ball.setPosition([-2,.2,3])

		self.ball.physNode.setVelocity(launchVel_XYZ) 
		
	def launchBall2(self):
		
		launchVel_XYZ = [ 0,0,3]
		self.ball.setPosition([-2,.2,-3])

		self.ball.physNode.setVelocity(launchVel_XYZ) 
	
	def launchBall3(self):
		
		launchVel_XYZ = [ -.2,0,-3 ]
		self.ball.setPosition([-2,.2,3])

		self.ball.physNode.setVelocity(launchVel_XYZ) 

	def dropBall(self):
		
		self.ball.setPosition(pos)
		self.ball.physNode.setVelocity([0,0,0]) 
		self.ball.physNode.enableMovement()





