"""
This script is to setup the ground plane and cave settings.
When it's completely functional, it will have the ability to display a ground plane, 
Allow functions to change basic configurations such as color, size, mode etc,
and also setup the cave frustrum and boundaries.

At this moment we define only one wall of the cave. Additional logic is required to 
define an entire CAVE (front, back, left, right, top and bottom walls). But this is 
as simple as just defining the additional wall boundries and adding them to the cave viwe.

Author:
Rahul N Gopinathan(@rnn4511)
"""
import viz
import vizcave
import vizact

#import viztracker
#import Connector
#import vizshape
#from Foot import Foot

"""
TODO: try threading for population of the buffer!!
"""

class virtualFloor():
	
	"""
	Initializes the Environment. Requires .cave file that contains the four corners of the powerwall.
	Also accepts the debug flag for future use.
	"""
	def __init__(self, config=None, planeName = 'floor', isAFloor = 1, caveCornersFileName = None):
		
		self.config = config
					
		self.cave = vizcave.Cave()
		
		self.caveCornersFileName = caveCornersFileName
		self.cave.load(self.caveCornersFileName)
		
		self.cornerCoordinates_cIdx = [0,0,0]*4

		
		#The powerwall, see vizard docs for info on powerwall
		self.powerwall = None
		
		self.planeName = planeName
		self.isAFloor = isAFloor
		
		### Define the cave powerwall boundries in the world###
		if(None == caveCornersFileName):
			self.powerwall = vizcave.Wall(  
								upperLeft = [-1,0,1],
								upperRight = [1,0,1],
								lowerRight = [1,0,-1],
								lowerLeft = [-1,0,-1],
								name = self.planeName )
		else:
			self.cave = vizcave.Cave()
			# This loads the contents of the file directly into the cave object
			self.cave.load(self.caveCornersFileName) #if None is not self.caveWallFileName else self.cave.addWall(self.powerwall)
									
	def setNewCornerPosition(self,cIdx,markerLocation_XYZ):
		# cIdx = clockwise from top left
		
		self.cornerCoordinates_cIdx[cIdx] = [-marker[2]/1000, marker[1]/1000, -marker[0]/1000]
		
		if( self.isAFloor ):
			print 'Updated corner of floor with height = 0'
			self.cornerCoordinates_cIdx[cIdx] = 0;
		else:
			print 'Updated corner.'
			
	def updatePowerwall(self):
		
		if( self.cave == none ):
			self.cave = vizcave.Cave()
		else:
			self.cave.removeWall(self.powerwall)
			self.cave.clear()
		
		self.powerwall = vizcave.Wall(  upperLeft = self.cornerCoordinates_cIdx[0],
								upperRight = self.cornerCoordinates_cIdx[1],
								lowerRight = self.cornerCoordinates_cIdx[2],
								lowerLeft = self.cornerCoordinates_cIdx[3],
								name = self.planeName )
								
		print "Updating powerwall to reflect new corner positions"
		
		self.cave.addWall(self.powerwall)
		self.cave.update()
		self.cave.save('caveWallDimensions.cave')

if __name__ == "__main__":	
	
	

	

	expConfigName = 'exampleExpConfig.cfg'
	import vrlabConfig
	config = vrlabConfig.VRLabConfig(expConfigName)
	
	#def __init__(self, config=None, planeName = 'floor', isAFloor = 1, caveCornersFileName = None, debug = True):
	env = virtualFloor(caveCornersFileName = 'caveWallDimensions.cave',config=config,planeName = 'floor', isAFloor = 1)
	
#	viz.window.setFullscreenMonitor(2)
#	viz.setMultiSample(8)
#	viz.go(viz.FULLSCREEN | viz.QUAD_BUFFER)
#	
	#room = room(config )
	viz.addChild('piazza.osgb')