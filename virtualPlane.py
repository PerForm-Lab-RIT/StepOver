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

import viztracker
#import Connector
#import vizshape
#from Foot import Foot

"""
TODO: try threading for population of the buffer!!
"""

class virtualPlane():
	
	"""
	Initializes the Environment. Requires .cave file that contains the four corners of the powerwall.
	Also accepts the debug flag for future use.
	"""
	def __init__(self, config=None, planeName = 'floor', isAFloor = 1, caveCornersFileName = None):
		
		self.config = config
					
		self.cave = vizcave.Cave()
		
		self.caveCornersFileName = caveCornersFileName
		self.cave.load(self.caveCornersFileName)
		
		self.cornerCoordinates_cIdx = [[0,0,0]] * 4

		#The powerwall, see vizard docs for info on powerwall
		self.powerwall = None
		
		self.planeName = planeName
		self.isAFloor = isAFloor
		
		### Define the cave powerwall boundries in the world###
		if(None == caveCornersFileName):
			print 'Using default caveCorners'
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

								
	def setNewCornerPosition(self,cIdx,markerNum):
		
		# cIdx = clockwise from top left
		markerLocation_XYZ = self.config.mocap.getMarkerPosition(markerNum)
		
		if( markerLocation_XYZ is None):
			print 'Marker data is no good!'
			return
		else:
			print str(markerLocation_XYZ)
			
		self.cornerCoordinates_cIdx[cIdx] = [markerLocation_XYZ[0], markerLocation_XYZ[1], markerLocation_XYZ[2]]
		
		if( self.isAFloor ):
			print 'Updated corner of floor with height = 0'
			self.cornerCoordinates_cIdx[cIdx][1] = 0;
		else:
			print 'Updated corner.'
			
	def updatePowerwall(self):
		
		if( self.cave == None):
			self.cave = vizcave.Cave()
		else:
			self.cave.removeWall(self.powerwall)
			self.cave.clear()
	

		self.powerwall = vizcave.Wall(  upperLeft = self.cornerCoordinates_cIdx[0],
								upperRight = self.cornerCoordinates_cIdx[1],
								lowerRight = self.cornerCoordinates_cIdx[2],
								lowerLeft = self.cornerCoordinates_cIdx[3],
								name = self.planeName)
								
		print "Updating powerwall to reflect new corner positions"
		
		self.cave.addWall(self.powerwall)
		self.cave.update()
		self.cave.save('caveWallDimensions.cave')

	def useDefaultView(self):
		
		"""
		The cave_origin node is a standard Vizard node that you can apply any position/rotation to.
		In this example we will create a keyboard/mouse tracker (using arrow keys) and link it to
		the cave_origin node, allowing us to  fly the cave user through the virtual environment.
		"""

		origin_tracker = viztracker.KeyboardMouse6DOF()
		origin_link = viz.link(origin_tracker, cave_origin)
		#origin_link.setMask(viz.LINK_POS)
		

		
		
	def attachViewToGlasses(self,visNode,glassesRigid):
		
		"""
		Create tracker object that represents the users head position, specifically the center of the eyes.
		The position provided by the head tracker must be in the same reference frame as the cave wall coordinates.
		This will normally be a tracking sensor, but for this example we will simulate a head tracker
		using the keyboard (WASD keys).
		"""
		self.head_tracker = viz.link(visNode,viz.NullLinkable,srcFlag=viz.ABS_PARENT)
		
		"""
		Create CaveView object for manipulating the virtual viewpoint.
		cave_origin is a node that controls the position of the cave within the virtual world.
		For example, if you wanted to simulate the cave user flying through an environment,
		you would apply the transformation to the cave_origin node.
		"""
		cave_origin = vizcave.CaveView(self.head_tracker)

		"""
		The cave_origin node is a standard Vizard node that you can apply any position/rotation to.
		In this example we will create a keyboard/mouse tracker (using arrow keys) and link it to
		the cave_origin node, allowing us to  fly the cave user through the virtual environment.
		"""

		origin_tracker = viztracker.KeyboardMouse6DOF()
		origin_link = viz.link(origin_tracker, cave_origin)
		origin_link.setMask(viz.LINK_POS)
		
		
		#head_tracker.setMask(viz.LINK_POS)

		
		"""
		Pass the head tracker to the cave object so it can automatically update the
		view frustums every frame based on the current head position relative to each wall.
		"""
		self.cave.setTracker(self.head_tracker)

	
	def getCenterPos(self,planeName='floor'):
		
		center_XYZ = [0,0,0]
		
		walls_wIdx = self.cave.getWalls()
		
		for wIdx in range(len(walls_wIdx)):
			# NOTE:  I have to do some strange formatting, b/c vizcave is returning an ugly string
			# To see what I mean, debug here, and check out: walls_wIdx[wIdx]._name
			wallName = str(walls_wIdx[wIdx]._name).lower()
			if( wallName  == planeName.lower()):
				center_XYZ = walls_wIdx[0].getCenter()
		
		return center_XYZ
		
if __name__ == "__main__":	
	
	expConfigName = 'exampleExpConfig.cfg'
	import vrlabConfig
	config = vrlabConfig.VRLabConfig(expConfigName)
	
	#def __init__(self, config=None, planeName = 'floor', isAFloor = 1, caveCornersFileName = None, debug = True):
	env = virtualPlane(caveCornersFileName = 'caveWallDimensions.cave',config=config,planeName = 'floor', isAFloor = 1)

	import visEnv
	#def __init__(self,room,shape,size,position=[0,.25,-3],color=[.5,0,0],alpha = 1):
	
	room = visEnv.room(config)
	room.walls.remove()
	
	viz.addChild('piazza.osgb')
	
	shutterRigid = config.mocap.returnPointerToRigid('shutter')
	
	eyeSphere = visEnv.visObj(room,'sphere',size=0.1,alpha=1)
	eyeSphere.setMocapRigidBody(config.mocap,'shutter')
	
	eyeSphere.toggleUpdateWithRigid()
	
	env.attachViewToGlasses(eyeSphere.visNode,shutterRigid)
	
	markerNum = 0;
		
	vizact.onkeydown('1', env.setNewCornerPosition,0,markerNum)
	vizact.onkeydown('2', env.setNewCornerPosition,1,markerNum)
	vizact.onkeydown('3', env.setNewCornerPosition,2,markerNum)
	vizact.onkeydown('4', env.setNewCornerPosition,3,markerNum)
	
	vizact.onkeydown('0', env.updatePowerwall)
	
	vizact.onkeydown('s', config.mocap.resetRigid, 'shutterGlass')
	vizact.onkeydown('S', config.mocap.saveRigid, 'shutterGlass')	
	
				
	wii = viz.add('wiimote.dle')#Add wiimote extension
		
	vizact.onsensorup(config.wiimote,wii.BUTTON_LEFT,env.setNewCornerPosition,0,markerNum)
	vizact.onsensorup(config.wiimote,wii.BUTTON_UP,env.setNewCornerPosition,1,markerNum)
	vizact.onsensorup(config.wiimote,wii.BUTTON_RIGHT,env.setNewCornerPosition,2,markerNum)
	vizact.onsensorup(config.wiimote,wii.BUTTON_DOWN,env.setNewCornerPosition,3,markerNum)
	vizact.onsensorup(config.wiimote,wii.BUTTON_PLUS,env.updatePowerwall)
	
	vizact.onsensorup(config.wiimote,wii.BUTTON_MINUS,viz.MainWindow.setStereoSwap,viz.TOGGLE)
	
	duckBeginPos =  env.getCenterPos()
	duck = viz.addAvatar('duck.cfg',pos=duckBeginPos)
	duck.scale([0.3,0.3,0.3])
	eyeSphere.setMocapRigidBody(config.mocap,'shutter')
	
	viz.MainWindow.setStereoSwap(viz.TOGGLE)