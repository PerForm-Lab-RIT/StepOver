"""
Runs an experiment.
"""

viz.res.addPath('resources')
sys.path.append('utils')

import viz
import viztask
import vizshape
import vizact
from drawNumberFromDist import *
import visEnv
import physEnv
import ode
import datetime
from ctypes import * # eyetrackka
import winsound
import virtualPlane
import vizinput

from obstacleClass import obstacleObj

#For hardware configuration
viz.res.addPath('resources')
sys.path.append('utils')
from configobj import ConfigObj
from configobj import flatten_errors
from validate import Validator
import platform
import os.path
import vizconnect



expConfigFileName = 'exampleExpConfig.cfg'

ft = .3048
inch = 0.0254
m = 1
eps = .01
nan = float('NaN')

# Create a globally accessible soundbank.
# To access within a member function, 
# import the global variable with 'global soundbank'

class soundBank():
	def __init__(self):
		 
		################################################################
		################################################################
		
		## Register sounds.  It makes sense to do it once per experiment.
		self.bounce =  '/Resources/bounce.wav'
		self.buzzer =  '/Resources/BUZZER.wav'
		self.bubblePop =  '/Resources/bubblePop3.wav'
		self.highDrip =  '/Resources/highdrip.wav'
		self.cowbell =  '/Resources/cowbell.wav'
		self.beep =  '/Resources/beep.wav'
		self.gong = '/Resources/gong.wav'
		
		viz.playSound(self.gong,viz.SOUND_PRELOAD)
		viz.playSound(self.beep,viz.SOUND_PRELOAD)
		viz.playSound(self.bounce,viz.SOUND_PRELOAD)
		viz.playSound(self.buzzer,viz.SOUND_PRELOAD)
		viz.playSound(self.bubblePop,viz.SOUND_PRELOAD)
		viz.playSound(self.highDrip,viz.SOUND_PRELOAD)
		viz.playSound(self.cowbell,viz.SOUND_PRELOAD)

soundBank = soundBank()

class Configuration():
	
	def __init__(self, expCfgName = ""):
		"""
		Opens and interprets both the system config (as defined by the <platform>.cfg file) and the experiment config
		(as defined by the file in expCfgName). Both configurations MUST conform the specs given in sysCfgSpec.ini and
		expCfgSpec.ini respectively. It also initializes the system as specified in the sysCfg.
		"""
		self.eyeTracker = []
		
		self.writables = list()
		if expCfgName:
			self.__createExpCfg(expCfgName)
		else:
			self.expCfg = None
			
		self.__createSysCfg()
		
		for pathName in self.sysCfg['set_path']:
			viz.res.addPath(pathName)
			
		self.vizconnect = vizconnect.go( 'vizConnect/' + self.sysCfg['vizconfigFileName'])
		self.__postVizConnectSetup()
		
	def __postVizConnectSetup(self):
		
		''' 
		This is where one can run any system-specific code that vizconnect can't handle
		'''

		dispDict = vizconnect.getRawDisplayDict()
		
		#self.clientWindow = dispDict['custom_window']
		#self.riftWindow = dispDict['rift']
		
		if self.sysCfg['use_phasespace']:
			
			from mocapInterface import phasespaceInterface			
			self.mocap = phasespaceInterface(self.sysCfg);
			
			self.use_phasespace = True
		else:
			self.use_phasespace = False

		if( self.sysCfg['use_wiimote']):
			# Create wiimote holder
			self.wiimote = 0
			self.__connectWiiMote()

		if self.sysCfg['use_hmd'] and self.sysCfg['hmd']['type'] == 'DK2':
			self.__setupOculusMon()
		
		if self.sysCfg['use_eyetracking']:
			self.use_eyetracking = True
			self.__connectSMIDK2()
		else:
			self.use_eyetracking = False
			
		if self.sysCfg['use_DVR'] == 1:
			self.use_DVR = True
		else:
			self.use_DVR = False
		
		if self.sysCfg['use_virtualPlane']:
			self.use_VirtualPlane = True
			
			isAFloor = self.sysCfg['virtualPlane']['isAFloor']
			planeName = self.sysCfg['virtualPlane']['planeName']
			planeCornerFile = self.sysCfg['virtualPlane']['planeCornerFile']
			
			self.virtualPlane = virtualPlane.virtualPlane(self,planeName,isAFloor,planeCornerFile)
			
		self.writer = None #Will get initialized later when the system starts
		self.writables = list()
		
		#__setWinPriority()
		#viz.setMultiSample(self.sysCfg['antiAliasPasses'])
		#viz.MainWindow.clip(0.01 ,200)
		
		#viz.vsync(1)
		#viz.setOption("viz.glfinish", 1)
		#viz.setOption("viz.dwm_composition", 0)
		
	def __createExpCfg(self, expCfgName):

		"""

		Parses and validates a config obj
		Variables read in are stored in configObj
		
		"""
		
		print "Loading experiment config file: " + expCfgName
		
		# This is where the parser is called.
		expCfg = ConfigObj(expCfgName, configspec='expCfgSpec.ini', raise_errors = True, file_error = True)

		validator = Validator()
		expCfgOK = expCfg.validate(validator)
		if expCfgOK == True:
			print "Experiment config file parsed correctly"
		else:
			print 'Experiment config file validation failed!'
			res = expCfg.validate(validator, preserve_errors=True)
			for entry in flatten_errors(expCfg, res):
			# each entry is a tuple
				section_list, key, error = entry
				if key is not None:
					section_list.append(key)
				else:
					section_list.append('[missing section]')
				section_string = ', '.join(section_list)
				if error == False:
					error = 'Missing value or section.'
				print section_string, ' = ', error
			sys.exit(1)
		if expCfg.has_key('_LOAD_'):
			for ld in expCfg['_LOAD_']['loadList']:
				print 'Loading: ' + ld + ' as ' + expCfg['_LOAD_'][ld]['cfgFile']
				curCfg = ConfigObj(expCfg['_LOAD_'][ld]['cfgFile'], configspec = expCfg['_LOAD_'][ld]['cfgSpec'], raise_errors = True, file_error = True)
				validator = Validator()
				expCfgOK = curCfg.validate(validator)
				if expCfgOK == True:
					print "Experiment config file parsed correctly"
				else:
					print 'Experiment config file validation failed!'
					res = curCfg.validate(validator, preserve_errors=True)
					for entry in flatten_errors(curCfg, res):
					# each entry is a tuple
						section_list, key, error = entry
						if key is not None:
							section_list.append(key)
						else:
							section_list.append('[missing section]')
						section_string = ', '.join(section_list)
						if error == False:
							error = 'Missing value or section.'
						print section_string, ' = ', error
					sys.exit(1)
				expCfg.merge(curCfg)
		
		self.expCfg = expCfg

	
	def __setWinPriority(self,pid=None,priority=1):
		
		""" Set The Priority of a Windows Process.  Priority is a value between 0-5 where
			2 is normal priority.  Default sets the priority of the current
			python process but can take any valid process ID. """
			
		import win32api,win32process,win32con
		
		priorityclasses = [win32process.IDLE_PRIORITY_CLASS,
						   win32process.BELOW_NORMAL_PRIORITY_CLASS,
						   win32process.NORMAL_PRIORITY_CLASS,
						   win32process.ABOVE_NORMAL_PRIORITY_CLASS,
						   win32process.HIGH_PRIORITY_CLASS,
						   win32process.REALTIME_PRIORITY_CLASS]
		if pid == None:
			pid = win32api.GetCurrentProcessId()
		
		handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, pid)
		win32process.SetPriorityClass(handle, priorityclasses[priority])
		
	def __createSysCfg(self):
		"""
		Set up the system config section (sysCfg)
		"""
		
		# Get machine name
		sysCfgName = platform.node()+".cfg"
		
		if not(os.path.isfile(sysCfgName)):
			sysCfgName = "defaultSys.cfg"
			
		print "Loading system config file: " + sysCfgName
		
		# Parse system config file
		sysCfg = ConfigObj(sysCfgName, configspec='sysCfgSpec.ini', raise_errors = True)
		
		validator = Validator()
		sysCfgOK = sysCfg.validate(validator)
		
		if sysCfgOK == True:
			print "System config file parsed correctly"
		else:
			print 'System config file validation failed!'
			res = sysCfg.validate(validator, preserve_errors=True)
			for entry in flatten_errors(sysCfg, res):
			# each entry is a tuple
				section_list, key, error = entry
				if key is not None:
					section_list.append(key)
				else:
					section_list.append('[missing section]')
				section_string = ', '.join(section_list)
				if error == False:
					error = 'Missing value or section.'
				print section_string, ' = ', error
			sys.exit(1)
		self.sysCfg = sysCfg
	
		
	def __setupOculusMon(self):
		"""
		Setup for the oculus rift dk2
		Relies upon a cluster enabling a single client on the local machine
		THe client enables a mirrored desktop view of what's displays inside the oculus DK2
		Note that this does some juggling of monitor numbers for you.
		"""
		
		#viz.window.setFullscreenMonitor(self.sysCfg['displays'])
		
		#hmd = oculus.Rift(renderMode=oculus.RENDER_CLIENT)

		displayList = self.sysCfg['displays'];
		
		if len(displayList) < 2:
			print 'Display list is <1.  Need two displays.'
		else:
			print 'Using display number' + str(displayList[0]) + ' for oculus display.'
			print 'Using display number' + str(displayList[1]) + ' for mirrored display.'
		
		### Set the rift and exp displays
		
		riftMon = []
		expMon = displayList[1]
		
		with viz.cluster.MaskedContext(viz.MASTER):
			
			# Set monitor to the oculus rift
			monList = viz.window.getMonitorList()
			
			for mon in monList:
				if mon.name == 'Rift DK2':
					riftMon = mon.id
			
			viz.window.setFullscreen(riftMon)

		with viz.cluster.MaskedContext(viz.CLIENT1):
			
			count = 1
			while( riftMon == expMon ):
				expMon = count
				
			viz.window.setFullscreenMonitor(expMon)
			viz.window.setFullscreen(1)

	def __connectWiiMote(self):
		
		wii = viz.add('wiimote.dle')#Add wiimote extension
		
		# Replace old wiimote
		if( self.wiimote ):
			print 'Wiimote removed.'
			self.wiimote.remove()
			
		self.wiimote = wii.addWiimote()# Connect to first available wiimote
		
		vizact.onexit(self.wiimote.remove) # Make sure it is disconnected on quit
		
		self.wiimote.led = wii.LED_1 | wii.LED_4 #Turn on leds to show connection
	
	def __connectSMIDK2(self):
		
		if self.sysCfg['sim_trackerData']:
			self.eyeTracker = smi_beta.iViewHMD(simulate=True)
		else:
			self.eyeTracker = smi_beta.iViewHMD()
	
	def __record_data__(self, e):
		
		if self.use_DVR and self.writer != None:
			#print "Writing..."
			self.writer.write(self.writables)
		
	def startDVR(self):
		
		if self.use_DVR:
			print "Starting DVR"
			from DVRwriter				import DVRwriter
			from datetime 				import datetime
			
			metadata = 'unused per-file meta data' #Can be filled in with useful metadata if desired.
			
			if None == self.writer: #need to lazy initialize this because it has to be called after viz.go()
				
				sz = viz.window.getSize()
				self.now = datetime.now()
				nameRoot = '%s%d.%d.%d.%d-%d' % (self.sysCfg['writer']['outFileDir'], self.now.year, self.now.month, self.now.day, self.now.hour, self.now.minute)
				outFile = '%s.%s' % (nameRoot, self.sysCfg['writer']['outFileName'])
				self.expCfg.filename = '%s.expCfg.cfg' % (nameRoot)
				self.sysCfg.filename = '%s.sysCfg.cfg' % (nameRoot)
				
				
#				if 'L' == self.sysCfg['eyetracker']['eye']: 
#					viewport = (0,      0,sz[0]/2,sz[1])
#				else:               
#					viewport = (sz[0]/2,0,sz[0]/2,sz[1])
#				#fi
				
				viewport = self.clientWindow
				viewPosXY = viewport.getPosition(viz.WINDOW_PIXELS)
				viewSizeXY = viewport.getSize(viz.WINDOW_PIXELS)
				
				#viewport = (1920,0,1920,1200)
				viewport = (0,0,1920,1200)
				
				
				print "OutfileName:" + self.sysCfg['writer']['outFileName']
				print "Metadata:" + metadata
				print "Viewport:" + str(viewport)
				print "Eyetracking:" + str(self.use_eyetracking)
				
			
				self.writer = DVRwriter(outFile, metadata, viewport,0)
				self.expCfg.write()
				self.sysCfg.write()
				
			self.writer.turnOn()
			


class Experiment(viz.EventClass):
	
	"""
	Experiment manages the basic operation of the experiment.
	"""
	
	def __init__(self, expConfigFileName):
		
		# Event classes can register their own callback functions
		# This makes it possible to register callback functions (e.g. activated by a timer event)
		# within the class (that accept the implied self argument)
		# eg self.callbackFunction(arg1) would receive args (self,arg1)
		# If this were not an eventclass, the self arg would not be passed = badness.
		
		viz.EventClass.__init__(self)
		
		##############################################################
		##############################################################
		## Use config to setup hardware, motion tracking, frustum, eyeTrackingCal.
		##  This draws upon the system config to setup the hardware / HMD
		
		#self.config = config
		config = Configuration(expConfigFileName)
		self.config = config
		
		# Update to reflect actual leg length of user
		self.inputLegLength()
		
		################################################################
		################################################################
		## Set states
		
		self.inCalibrateMode = False
		self.inHMDGeomCheckMode = False
		self.setEnabled(False)
		self.test_char = None

		################################################################
		################################################################
		# Create visual and physical objects (the room)
	
		self.room = visEnv.room(config)
		viz.phys.enable()
		
		# self.room.physEnv 
		self.hmdLinkedToView = False		
		
		
		self.directionArrow = visEnv.visObj(self.room,'arrow',[.1,.1,.1],[-.8,.2,0],viz.PURPLE)
		self.directionArrow.setEuler([270,0,0])
		################################################################
		################################################################
		# Build block and trial list
		
		self.blockNumber = 0;
		self.trialNumber = 0;
		self.expInProgress = True;
		
		self.totalTrialNumber = 0;
		self.blocks_bl = []
		
		self.room.offsetDistance = float(config.expCfg['room']['minObstacleDistance'])
		print'====> Obstacle Distance = ', self.room.offsetDistance
		for bIdx in range(len(config.expCfg['experiment']['blockList'])):
			self.blocks_bl.append(block(config,bIdx, self.room));
		
		self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
		
#		################################################################
#		################################################################
#		##  Misc. Design specific items here.
		self.maxTrialDuration = config.expCfg['experiment']['maxTrialDuration']
		# Initially, set to config value of leg length
		config.legLengthCM = config.expCfg['experiment']['legLengthCM']
		
		
		
		if( config.wiimote ):
			self.registerWiimoteActions()

		self.obstacleViewTimerID = viz.getEventID('obstacleViewTimerID') # Generates a unique ID. 
		
		self.numClicksBeforeGo = config.expCfg['experiment']['numClicksBeforeGo']
		self.trialEndPosition = config.expCfg['experiment']['trialEndPosition']
		self.metronomeTimeMS = config.expCfg['experiment']['metronomeTimeMS']
				
		##  Setup virtual plane
		if( self.config.use_phasespace == True and self.config.sysCfg['virtualPlane']['attachGlassesToRigid']):
		
			#eyeSphere = visEnv.visObj(self.room,'sphere',size=0.1,alpha=1)
			#eyeSphere.visNode.setParent(self.room.objects)
			
			self.setupShutterGlasses()
			self.setupFeet()
			
		##############################################################
		##############################################################
		## Callbacks and timers
		
		# Hacked (Kamran)
		vizact.onupdate(viz.PRIORITY_PHYSICS,self._checkForCollisions)
		
		self.callback(viz.KEYDOWN_EVENT,  self.onKeyDown)
		self.callback(viz.KEYUP_EVENT, self.onKeyUp)
		self.callback( viz.TIMER_EVENT,self._timerCallback )
	
		
		self.perFrameTimerID = viz.getEventID('perFrameTimerID') # Generates a unique ID.
		self.starttimer( self.perFrameTimerID, viz.FASTEST_EXPIRATION, viz.FOREVER)
		
		self.trialTimeoutTimerID = viz.getEventID('trialTimeoutTimerID') # Generates a unique ID.
		
		# DVR snaps a shot of the frame, records eye data, and contents of self.writables is written out to the movie
		self.callback(viz.POST_SWAP_EVENT, self.config.__record_data__, viz.PRIORITY_LAST_UPDATE)
	
		
		# Use text output
		now = datetime.datetime.now()
		dateTimeStr = str(now.year) + '-' + str(now.month) + '-' + str(now.day) + '-' + str(now.hour) + '-' + str(now.minute)
		
		dataOutPutDir = config.sysCfg['writer']['outFileDir']
		
		self.expDataFile = open(dataOutPutDir + 'exp_data-' + dateTimeStr + '.txt','w+')
		self.writeOutDataFun = vizact.onupdate(viz.PRIORITY_LAST_UPDATE, self.writeDataToText)

# TODO: Double check if this is needed for future or not (Kamran)
#		# Use text output!
#		if( config.sysCfg['use_DVR'] > 0 ):
#			
#			vizact.ontimer(3,self.checkDVRStatus)
#			
#			now = datetime.datetime.now()
#			dateTimeStr = str(now.year) + '-' + str(now.month) + '-' + str(now.day) + '-' + str(now.hour) + '-' + str(now.minute)
#			
#			dataOutPutDir = config.sysCfg['writer']['outFileDir']
#			
#			self.expDataFile = open(dataOutPutDir + 'exp_data-' + dateTimeStr + '.txt','a')
#			
#			if( self.config.sysCfg['use_eyetracking']):
#				self.eyeDataFile = open(dataOutPutDir + 'eye_data-' + dateTimeStr + '.txt','a')
#			
#			vizact.onupdate(viz.PRIORITY_LAST_UPDATE,self.writeDataToText)
		
		# Create an event flag object
		# This var is set to an int on every frame
		# The int saves a record of what was happening on that frame
		# It can be configured to signify the start of a trial, the bounce of a ball, or whatever
		
		self.eventFlag = eventFlag()

	def _timerCallback(self,timerID):

		mainViewPos_XYZ = viz.MainView.getPosition()
		
		## This is all the per-frame timer stuff
		if( self.currentTrial.approachingObs == True and
			mainViewPos_XYZ[0] > self.trialEndPosition ):
			print 'Passed distance threshold.  Ending trial'
			self.endTrial()
				
		######################################################################
		## Are the feet in the starting position?
			
		if( self.currentTrial.approachingObs == False ):
			
			if( self.isVisObjInBox(self.room.leftFoot) and self.isVisObjInBox(self.room.rightFoot) ):
				self.currentTrial.subIsInBox = True
			
			else:
				self.currentTrial.subIsInBox = False

			######################################################################
			## If foot is in box, present obstacle and start metronome
			
			if( self.currentTrial.subIsInBox is True and 
				self.currentTrial.waitingForGo is False ):
					
				# Begin lockout period
				#print 'Subject is ready and waiting in the box. Present the obstacle.'
				
				# Yes, the head is inside the standing box
				self.currentTrial.waitingForGo = True
				# Present the obstacle
				self.currentTrial.placeObs(self.room)
				
				# Metronome has been deactivated
				#if( type(self.currentTrial.metronomeTimerObj) is list ):
					# Start a metronome that continues for the duration of the trial
					#self.currentTrial.metronomeTimerObj = vizact.ontimer2(self.metronomeTimeMS/1000, self.numClicksBeforeGo,self.metronomeLowTic)
				
				timeUntilGoSignal = ((self.numClicksBeforeGo)*self.metronomeTimeMS)/1000
				
				# Start the go signal timer
				if( type(self.currentTrial.goSignalTimerObj) is list ):
					# Start a metronome that continues for the duration of the trial
					self.currentTrial.goSignalTimerObj = vizact.ontimer2(timeUntilGoSignal, 0,self.giveGoSignal)

			########################################################################
			## Subject has just left the box
			elif( self.currentTrial.subIsInBox is False and 
				self.currentTrial.waitingForGo is True):
				
				########################################
				### Subject left before go signal was given!
				if(self.currentTrial.goSignalGiven is False ):
					  
					# Head was removed from box after viewing was initiated
					print 'Left box prematurely!'
					
					viz.playSound(soundBank.cowbell);
					
					# Remove box
					self.currentTrial.waitingForGo = False
					self.currentTrial.removeObs();
					
					#self.currentTrial.metronomeTimerObj.setEnabled(viz.TOGGLE);
					#self.currentTrial.metronomeTimerObj = [];
				
					self.currentTrial.goSignalTimerObj.setEnabled(viz.TOGGLE);
					self.currentTrial.goSignalTimerObj = [];
				
				##############################################
				### Go signal already given.  Starting the trial
				elif(self.currentTrial.goSignalGiven is True):
					
					print 'Starting trial ==> Type', self.currentTrial.trialType
					self.eventFlag.setStatus(1)
					self.currentTrial.approachingObs = True
					self.eventFlag.setStatus(6)
					
					# Start data collection
					viz.playSound(soundBank.bubblePop)
					
					if( type(self.currentTrial.goSignalTimerObj) is not list ):			
						self.currentTrial.goSignalTimerObj.remove()
	
					vizact.ontimer2(self.maxTrialDuration, 0,self.endTrial)
					
										
					#self.maxTrialDurationObj
					#self.trialTimeoutTimerID
				
	def _checkForCollisions(self):
		
		thePhysEnv = self.room.physEnv;
		
		if( ( thePhysEnv.collisionDetected == False ) or ( self.expInProgress == False ) ): 
			# No collisions this time!
			return
		
		leftFoot = self.room.leftFoot
		rightFoot = self.room.rightFoot
		
		if( self.currentTrial.approachingObs == True ):
			
			obstacle = self.currentTrial.obsObj.collisionBox
			
			for idx in range(len(thePhysEnv.collisionList_idx_physNodes)):
				
				physNode1 = thePhysEnv.collisionList_idx_physNodes[idx][0]
				physNode2 = thePhysEnv.collisionList_idx_physNodes[idx][1]
				
				if( physNode1 == leftFoot.physNode and physNode2 == obstacle.physNode):

					self.eventFlag.setStatus(4)
					
					bouncePos_XYZ,normal,depth,geom1,geom2 = thePhysEnv.contactObjects_idx[0].getContactGeomParams()
					self.collisionLocOnObs_XYZ = obstacle.physNode.collisionPosLocal_XYZ
					
					#print 'Frame: ' + str(viz.getFrameNumber()) + ' collision at: ' + str(self.collisionLocOnObs_XYZ)
					
					#print 'Collided Objects are Left Foot and Obstacle at\n', self.collisionLocOnObs_XYZ
					#self.currentTrial.removeObs()
					viz.playSound(soundBank.beep)
				
				elif( physNode1 == rightFoot.physNode and physNode2 == obstacle.physNode ):
					
					self.eventFlag.setStatus(5)
					bouncePos_XYZ,normal,depth,geom1,geom2 = thePhysEnv.contactObjects_idx[0].getContactGeomParams()

					self.collisionLocOnObs_XYZ = obstacle.physNode.collisionPosLocal_XYZ
					
					#print 'Collided Objects are Right Foot and Obstacle at\n', self.collisionLocOnObs_XYZ
					#self.currentTrial.removeObs()
					viz.playSound(soundBank.beep)
			
	def start(self):
		
		##This is called when the experiment should begin.
		self.setEnabled(True)

	def toggleEyeCalib(self):
		"""
		Toggles the calibration for eye tracking.
		Note, that for this to work, toggling 
		# self.config.camera must turn off your world model
		# This is setup in testRoom.init().
		
		# Example of what's needed in testRoom.init
		self.room = viz.addGroup()
		self.model = viz.add('pit.osgb',parent = self.room)
		"""
		
		if not self.config.mocap:
			pass
		#elif (self.config.mocap.isOn() == 1): 
		elif( self.config.mocap.mainViewUpdateAction == False):
			#self.config.mocap.turnOff()
			self.config.mocap.enableHMDTracking()
			#self.config.mocap.turnOn()
		else:
			self.config.mocap.disableHMDTracking()
			
		viz.mouse.setOverride(viz.TOGGLE)
		
		if( self.config.sysCfg['use_eyetracking'] ):
			self.config.eyeTrackingCal.toggleCalib()
		
		self.inCalibrateMode = not self.inCalibrateMode
		
		if self.inCalibrateMode:
			viz.clearcolor(.5, .5, .5)
			viz.MainView.setPosition(0,0,0)
			viz.MainView.setAxisAngle(0, 1, 0, 0)
			viz.MainView.velocity([0,0,0]);
			
		else:
			viz.clearcolor(0, 0, 0)
			
		if self.room:
			#self.room.visible(viz.disable)
			self.room.walls.visible(viz.TOGGLE)
			self.room.objects.visible(viz.TOGGLE)
		
	def createCamera(self):
		"""
		Head camera is generally initialized as part of the system calls. Additional changes should be added here.
		"""
		pass		


	def onKeyDown(self, key):
		"""
		Interactive commands can be given via the keyboard. Some are provided here. You'll likely want to add more.
		"""
		
		mocapSys = self.config.mocap;
		
		
		if key == 't':
			self.toggleWalkingDirection()
		###################R#######################################
		##########################################################
		## Keys used in the defauRRlt mode
		
		if( 'c' == key and self.config.sysCfg['use_eyetracking'] ):
			self.toggleEyeCalib()
			# a bit of a hack.  THe crossahair / calib ponit in viewpoint mapping is a bit off
			# until you hit a key.  So, I'm doing that for you.
			
			self.config.eyeTrackingCal.updateOffset('s')
			self.config.eyeTrackingCal.updateOffset('w')
		
#		elif( 'l' == key):
#			self.inputLegLength()
#			
		if (self.inCalibrateMode is False):
			if key == 'D':
				
				dvrWriter = self.config.wRRriter;
				dvrWriter.toggleOnOff()
				
#			if key == 'M':
#				
#				# Toggle the link between the HMD and Mainview
#				if( mocapSys ):
#					if( mocapSys.mainViewUpdateAction ):
#						mocapSys.disableHMDTracking()
#					else:
#						mocapSys.enableHMDTracking()
			
			elif key == 'S':
				mocapSys.resetRigid('shutter')
			elif key == 'L':
				mocapSys.resetRigid('left')
				self.resizeFootBox('left')
			elif key == 'R':
				mocapSys.resetRigid('right')
				self.resizeFootBox('right')
					
			if( viz.key.isDown( viz.KEY_CONTROL_L )):
				
				if key == 's':
					mocapSys.saveRigid('shutter')
				elif key == 'l':
					mocapSys.saveRigid('left')
				elif key == 'r':
					mocapSys.saveRigid('right')
			
		##########################################################
		##########################################################
		## Eye-tracker calibration mode
		
		if self.inCalibrateMode:
			if key == 'w':
				self.config.eyeTrackingCal.updateOffset('w')
			elif key == 's':
				self.config.eyeTrackingCal.updateOffset('s')
			elif key == 'i':
				self.config.eyeTrackingCal.updateDelta('w')
			elif key == 'k':
				self.config.eyeTrackingCal.updateDelta('s')
			elif key == 'j':
				self.config.eyeTrackingCal.updateDelta('a')
			elif key == 'l':
				self.config.eyeTrackingCal.updateDelta('d')
	
	
	def onKeyUp(self,key):
		
		if( key == 'v'):
			pass
			
			#self.launchKeyUp()

	def getOutput(self):
		
		
		"""
		Returns a string describing the current state of the experiment, useful for recording.
		"""
		# Fix Me:
		# When the markers are not visible it should not through Error
		
		# Legend:
		# ** for 1 var
		# () for 2 vars
		# [] for 3 vars
		# <> for 4 vars
		# @@ for 16 vars (view and projection matrices)
		
		#### Eventflag
		# 1 ball launched
		# 3 ball has hit floor 
		# 4 ball has hit paddle
		# 5 ball has hit back wall
		# 6 ball has timed out
		

		## =======================================================================================================
		## FrameTime, Event Flag, Trial Type 
		## =======================================================================================================				
		outputString = 'frameTime %f ' % (viz.getFrameTime())
		#87
		
		outputString = outputString + ' eventFlag %d ' % (self.eventFlag.status)
		
		#FIXME: Collision locatoin is not set correctly in _checkForCollisions
		#if( self.eventFlag.status == 4 or self.eventFlag.status == 5 ):
		
		### (Kamran) Hacked for test
		###  collisionPosLocal_XYZ = self.currentTrial.collisionPosLocal_XYZ
		###  outputString = outputString + ' collisionPosLocal_XYZ [ %f %f %f ] ' % (collisionPosLocal_XYZ[0], collisionPosLocal_XYZ[1], collisionPosLocal_XYZ[2])
		
		outputString = outputString + ' trialType %s ' % (self.currentTrial.trialType)

		## =======================================================================================================
		## Obstacle Height & Location 
		## =======================================================================================================
		outputString = outputString + '[ Obstacle_XYZ %f %f %f ] ' % ( self.currentTrial.obsXLoc, self.currentTrial.obsHeightM, self.currentTrial.obsZLoc)
		
		## =======================================================================================================
		## ViewPos 
		## =======================================================================================================				
		#viewPos_XYZ = viz.MainView.getPosition()
		#outputString = outputString + '[ viewPos_XYZ %f %f %f ] ' % (viewPos_XYZ[0],viewPos_XYZ[1],viewPos_XYZ[2])
		
		MarkerPos = []
		
		#FIXME: Hardcoded foot markers numbers for data output.

		self.config.sysCfg['phasespace']['owlParamMarkerCount']
		
		for i in range(17):
			
			markerCondition = self.config.mocap.getMarkerCondition(i)
			
			if( markerCondition > 0 ):
				Pos = self.config.mocap.getMarkerPosition(i);
				MarkerPos.append(Pos);
			else:
				MarkerPos.append([nan, nan, nan]);
		
		outputString = outputString + '< ShutterGlass_XYZ '
		# TODO: This should be stored in the ShutterGlass Object as .NumberOfMarker
		for i in range(5):
			ShutterGlassMarker = MarkerPos[i];
			outputString = outputString + 'G%d ' %(i)
			#try:
			outputString = outputString + '[ %f %f %f ] ' % (ShutterGlassMarker[0], ShutterGlassMarker[1], ShutterGlassMarker[2])		
			#except:
			#	a = 1;
		outputString = outputString + '> '
		
		outputString = outputString + '< RightFoot_XYZ '
		
		# TODO: This should be stored in the RightFootMarker Object as .NumberOfMarker
		for i in range(4):
			
			RightFootMarker = MarkerPos[i + 5];
			
			outputString = outputString + 'R%d ' %(i)
			try:
				outputString = outputString + '[ %f %f %f ] ' % (RightFootMarker[0], RightFootMarker[1], RightFootMarker[2])		
			except:
				a=1
		outputString = outputString + '> '

		outputString = outputString + '< LeftFoot_XYZ '
		# TODO: This should be stored in the LeftFootMarker Object as .NumberOfMarker
		for i in range(4):
			LeftFootMarker = MarkerPos[i + 9];
			outputString = outputString + 'L%d ' %(i)
			outputString = outputString + '[ %f %f %f ] ' % (LeftFootMarker[0], LeftFootMarker[1], LeftFootMarker[2])	

		outputString = outputString + '> '
		outputString = outputString + '< Spinal_XYZ '
		
		# TODO: This should be stored in the SpinalMarker Object as .NumberOfMarker
		for i in range(4):
			SpinalMarker = MarkerPos[i + 13];
			outputString = outputString + 'S%d ' %(i)
			outputString = outputString + '[ %f %f %f ] ' % (SpinalMarker[0], SpinalMarker[1], SpinalMarker[2])		
		outputString = outputString + '> '

		## =======================================================================================================
		## Left and Right Foot Position & Quaternion
		## =======================================================================================================
		rightFootPos_XYZ = []
		rightFootQUAT_XYZW = []
		leftFootPos_XYZ = []
		leftFootQUAT_XYZW = []

		# TODO: We can calculate each foot Velocity here. (Instead of doing it later offline)
		#rightFootVel_XYZ = []
		#leftFootVel_XYZ = []
		
		if( self.room.rightFoot ):
			
			rightFootPos_XYZ = self.room.rightFoot.visNode.getPosition()
			rightFootMat = self.room.rightFoot.visNode.getMatrix()
			rightFootQUAT_XYZW = rightFootMat.getQuat()
			
		else:
			rightFootPos_XYZ = [None, None, None]
			rightFootQUAT_XYZW = [None, None, None]
			
		#outputString = outputString + '[ RightFoot_XYZ %f %f %f ] ' % (rightFootPos_XYZ[0], rightFootPos_XYZ[1], rightFootPos_XYZ[2])		
		outputString = outputString + '[ RightFootQUAT_XYZW %f %f %f %f ] ' % ( rightFootQUAT_XYZW[0], rightFootQUAT_XYZW[1], rightFootQUAT_XYZW[2], rightFootQUAT_XYZW[3] )

		if( self.room.leftFoot ):
			
			leftFootPos_XYZ = self.room.leftFoot.visNode.getPosition()
			leftFootMat = self.room.leftFoot.visNode.getMatrix()
			leftFootQUAT_XYZW = leftFootMat.getQuat()
			
		else:
			leftFootPos_XYZ = [None, None, None]
			leftFootQUAT_XYZW = [None, None, None]
			
		#outputString = outputString + '[ LeftFoot_XYZ %f %f %f ] ' % (leftFootPos_XYZ[0], leftFootPos_XYZ[1], leftFootPos_XYZ[2])
		
		outputString = outputString + '[ LeftFootQUAT_XYZW %f %f %f %f ] ' % ( leftFootQUAT_XYZW[0], leftFootQUAT_XYZW[1], leftFootQUAT_XYZW[2], leftFootQUAT_XYZW[3] )
		outputString = outputString + ' WalkingDirection %d ' % (self.room.isWalkingUpAxis)
		
		return outputString #%f %d' % (viz.getFrameTime(), self.inCalibrateMode)
		
	
	def toggleWalkingDirection(self):
		print 'Changing Direction From ' + str(self.room.isWalkingUpAxis)+' to ' + str(not(self.room.isWalkingUpAxis))
		self.room.isWalkingUpAxis = not(self.room.isWalkingUpAxis)
		
		if( self.room.isWalkingUpAxis ):
			self.directionArrow.setEuler([90,0,0])
			self.room.standingBox.setPosition([0.0, 0.1, 2.4 ])
			
			#self.room.standingBox.setPosition([-0.1, 0.01, 1.7])
			# 2.6 to the front
			
		else:
			self.directionArrow.setEuler([270,0,0])
			self.room.standingBox.setPosition([0.0, 0.1, -3.])
			#self.room.standingBox.setPosition([-0.1, 0.01, -2.4])
			# 3.8 to the back
		
		
	def endTrial(self):
		
		endOfTrialList = len(self.blocks_bl[self.blockNumber].trials_tr)
		
		self.toggleWalkingDirection();	
		self.currentTrial.approachingObs = False
		#print 'Ending block: ' + str(self.blockNumber) + 'trial: ' + str(self.trialNumber)
		
		if( self.trialNumber < endOfTrialList ):
			
			recalAfterTrial_idx = self.blocks_bl[self.blockNumber].recalAfterTrial_idx
			
			if( recalAfterTrial_idx.count(self.trialNumber ) > 0):
				soundBank.gong.play()
				vizact.ontimer2(0,0,self.toggleEyeCalib)
				
			# Increment trial 
			self.trialNumber += 1
			
			## Play sound
			viz.playSound(soundBank.cowbell)
			## Remove obstacle
			#print 'End of Trial :> Attempting to remove Obs'
			self.currentTrial.removeObs()
			#print 'End of Trial :> Remove Obs'
		
			## Stop timers
			if( type(self.currentTrial.metronomeTimerObj) is not list ):			
				self.currentTrial.metronomeTimerObj.remove()
			
			if( type(self.currentTrial.goSignalTimerObj) is not list ):			
				self.currentTrial.goSignalTimerObj.remove()
			
			if( type(self.currentTrial.trialTimeoutTimerObj) is not list ):			
				self.currentTrial.trialTimeoutTimerObj.remove()
			
			
		if( self.trialNumber == endOfTrialList ):
			
			# Increment block
			
			# arg2 of 1 allows for overwriting eventFlag 6 (new trial)
			self.eventFlag.setStatus(7,True) 
			
			self.blockNumber += 1
			self.trialNumber = 0
			
			# End experiment
			if( self.blockNumber == len(self.blocks_bl) ):
				
				# Run this once on the next frame
				# This maintains the ability to record one frame of data
				vizact.ontimer2(0,0,self.endExperiment)
				return
				
		if( self.expInProgress ):
				
			print 'Starting block: ' + str(self.blockNumber) + ' Trial: ' + str(self.trialNumber)
			self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
			
	def writeDataToText(self):

		# Only write data if the experiment is ongoing
		if( (self.currentTrial.approachingObs == False) or (self.expInProgress == False) ):
			return
	
		expDataString = self.getOutput()
		self.expDataFile.write(expDataString + '\n')
	
	def registerWiimoteActions(self):
				
		wii = viz.add('wiimote.dle')#Add wiimote extension
		
		#vizact.onsensordown(self.config.wiimote,wii.BUTTON_B,self.launchKeyDown) 
		#vizact.onsensorup(self.config.wiimote,wii.BUTTON_B,self.launchKeyUp) 
		
		if( self.config.use_phasespace == True ):
			
			mocapSys = self.config.mocap;
		
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_1,mocapSys.resetRigid,'shutter') 
			
			env = self.config.virtualPlane
			markerNum  = self.config.sysCfg['virtualPlane']['recalibrateWithMarkerNum'] 
			
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_LEFT,env.setNewCornerPosition,0,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_UP,env.setNewCornerPosition,1,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_RIGHT,env.setNewCornerPosition,2,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_DOWN,env.setNewCornerPosition,3,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_PLUS,env.updatePowerwall)
			
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_MINUS,viz.MainWindow.setStereoSwap,viz.TOGGLE)
			#vizact.onsensorup(self.config.wiimote,wii.BUTTON_LEFT,mocapSys.resetRigid,'paddle') 
			#vizact.onsensorup(self.config.wiimote,wii.BUTTON_RIGHT,mocapSys.saveRigid,'paddle') 

		
		
	def endExperiment(self):
		# If recording data, I recommend ending the experiment using:
		#vizact.ontimer2(.2,0,self.endExperiment)
		# This will end the experiment a few frame later, making sure to get the last frame or two of data
		# This could cause problems if, for example, you end the exp on the same that the ball dissapears
		# ...because the eventflag for the last trial would never be recorded
		
		#end experiment
		# TODO: Make sure this is the correct place to close and flush the Text File
		self.expDataFile.flush()
		self.expDataFile.close()
		print 'End of Trial & Block ==> TxT file Saved & Closed'
		print 'end experiment'
		self.expInProgress = False
		viz.playSound(soundBank.gong)
			
	
	def checkDVRStatus(self):
	
		dvrWriter = self.config.writer;
		
		if( dvrWriter.isPaused == 1 ):
			print '************************************ DVR IS PAUSED ************************************'

	
	def metronomeLowTic(self):
		viz.playSound(soundBank.bubblePop)
	
	def metronomeHighTic(self):
		viz.playSound(soundBank.highDrip)
	
	def giveGoSignal(self):
		print 'Go signal given!'
		self.currentTrial.goSignalGiven = True
		
		if( type(self.currentTrial.metronomeTimerObj) is not list ):			
			self.currentTrial.metronomeTimerObj.remove()

		viz.playSound(soundBank.beep)
		# Change the Obstacle Color to Green as a visual feedback for the subject
		self.currentTrial.obsObj.setColor(viz.WHITE)
		
		#self.currentTrial.metronomeTimerObj = vizact.ontimer(self.metronomeTimeMS/1000,self.metronomeHighTic)
		
	def inputLegLength(self):
		#print('SETTING LEG LENGTH TO 100!')
		#self.config.obsHeightLegRatio = 100

		#prompt for a string
		#self.config.obsHeightLegRatio 
		self.config.legLengthCM = vizinput.input('Enter leg length (cm):', value = float(90))
		
		try:
			# Test if it's an integer
			intValue = int(self.config.legLengthCM)
		except ValueError:
			viz.message( 'Please enter a valid integer')
			self.inputLegLength()
	
	def isVisObjInBox(self,vizObj):
		
		pos_xyz = vizObj.visNode.getPosition()

		standingBoxOffsetX = self.config.expCfg['room']['standingBoxOffset_X']
		standingBoxOffsetZ = self.config.expCfg['room']['standingBoxOffset_Z']
		BBox = self.room.standingBox.getPosition();
		standingBoxOffsetZ = BBox[2]
		standingBoxSize_WHL = self.config.expCfg['room']['standingBoxSize_WHL']
		
			
		# Is the head inside the standing box?
		if( pos_xyz[0] > (standingBoxOffsetX - standingBoxSize_WHL[0]/2) and 
			pos_xyz[0] < (standingBoxOffsetX + standingBoxSize_WHL[0]/2) and
			pos_xyz[2] > (standingBoxOffsetZ - standingBoxSize_WHL[2]/2) and 
			pos_xyz[2] < (standingBoxOffsetZ + standingBoxSize_WHL[2]/2)):
		
			return 1
		else:
			return 0
					
	def setupShutterGlasses(self):

		config = self.config
		print 'Connecting mainview to eyesphere'

		viz.MainWindow.setStereoSwap(viz.TOGGLE)
		
		eyeSphere = self.room.eyeSphere
		eyeSphere.setMocapRigidBody(config.mocap,'shutter')
		eyeSphere.toggleUpdateWithRigid()
		eyeSphere.visNode.visible(viz.TOGGLE)
		
		shutterRigid = config.mocap.returnPointerToRigid('shutter')
		self.config.virtualPlane.attachViewToGlasses(eyeSphere.visNode,shutterRigid)
				
	def setupFeet(self):
		
		config = self.config
		leftFoot = self.room.leftFoot
		leftFoot.visNode.color([0, 0, .3])
		leftFoot.setMocapRigidBody(config.mocap,'leftFoot')
		leftFoot.toggleUpdateWithRigid()
		leftFoot.enablePhysNode()
		leftFoot.toggleUpdatePhysWithVis()
		#leftFoot.visNode.disable(viz.RENDERING)
		leftFoot.visNode.visible(viz.ON)
		
		rightFoot = self.room.rightFoot
		rightFoot.visNode.color([0.5, 0, 0])
		rightFoot.setMocapRigidBody(config.mocap,'rightFoot')
		rightFoot.toggleUpdateWithRigid()
		rightFoot.enablePhysNode()
		rightFoot.toggleUpdatePhysWithVis()
		#rightFoot.visNode.disable(viz.RENDERING)
		rightFoot.visNode.visible(viz.ON)

	def resizeFootBox(self,footSide):
		''' 
		Algorithm is:100
		100
		
			* Get all markers
			* Find marker at max(X axis)
			* Find marker at min(X axis)
			* Find middle of rigid body (it may be offset from mean of all marker locations)
			* Set box length 
			* Set width to 2x current width
			* Set height offset so that bottom of box is resting on groundplane

		'''
		if( footSide != 'left' and footSide != 'right'):
	
			print 'resizeFootBox: invalid foot size.  Accepted values are left, or right'
			return
			
		mocap = self.config.mocap
		
		# An array of 3 element arrays
		# Position is in mm, and in phasespace coordintes
		#markersOnRigidBody_mIdx_psXYZ = mocap.getRigidMarkerPositions(footSide) 
		#markersOnRigidBody_mIdx_vizXYZ = []

		
		#markerZVals_mIdx = []
		#markerXVals_mIdx= []
		
		footHeightFromGround = 0 
		footRigid = mocap.returnPointerToRigid(footSide)
	
		markerPosViz_mIdx_XYZ = mocap.getRigidMarkerPositions(footSide)

		markerXVals_mIdx = [markerPosViz_mIdx_XYZ[mIdx][0] for mIdx in range(len(markerPosViz_mIdx_XYZ))]
		markerZVals_mIdx = [markerPosViz_mIdx_XYZ[mIdx][2] for mIdx in range(len(markerPosViz_mIdx_XYZ))]

		if( markerPosViz_mIdx_XYZ == -1 ): # or len(markerPosViz_mIdx_XYZ) < expected number
			print 'Error: Could not see all foot markers'
			return
			
		sumOfMarkerHeights = 0
		
		for mIdx in footRigid.avgMarkerList_midx:
			sumOfMarkerHeights = sumOfMarkerHeights + markerPosViz_mIdx_XYZ[mIdx][1]
			
		avgMarkerHeight = sumOfMarkerHeights / len(footRigid.avgMarkerList_midx)
		
		# one centimeter  buffer
		footLength = max(markerXVals_mIdx) - min(markerXVals_mIdx)
		footHeight = avgMarkerHeight
		footWidth = (max(markerZVals_mIdx) - min(markerZVals_mIdx))
		
		footLWH = [footLength, footWidth, footHeight]
		
		if( footSide == 'left' ):			
			footObj = self.room.leftFoot
		
		elif( footSide == 'right' ):
			footObj= self.room.rightFoot
		
		# Return the length, width and height
		#footVizNode.size(
		footObj.size = footLWH
		
		print 'Making basic visNode of size ' + str(footLWH)
		footObj.visNode.remove()
		footObj.makeBasicVisNode()
		
		#self.setupEyesAndFeet()
			
class eventFlag(viz.EventClass):
	
	def __init__(self):
		
		################################################################
		##  Eventflag
		
		# 1 Trial Start
		# 2 
		# 3 
		# 4 Right foot collides with obstacle
		# 5 Left foot collides with obstacle
		# 6 Trial end
		# 7 Block end
		
		viz.EventClass.__init__(self)
		
		self.status = 0
		self.lastFrameUpdated = viz.getFrameNumber()
		self.currentValue = 0
		
		# On every frame, self.eventFlag should be set to 0
		# This should happen first, before any timer object has the chance to overwrite it!
		vizact.onupdate(viz.PRIORITY_FIRST_UPDATE,self._resetEventFlag)
		
	def setStatus(self,status,overWriteBool = False):
		
		if( self.lastFrameUpdated != viz.getFrameNumber() ):
			
			#print 'Setting status to ' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
			
		elif( overWriteBool is True and self.lastFrameUpdated == viz.getFrameNumber() ):
			
			#print 'Overwrite from status ' + str(self.status) + ' to ' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
		
			
		elif( self.lastFrameUpdated == viz.getFrameNumber() ):
			pass
			#print 'Stopped attempt to overwrite status of ' + str(self.status) + ' with ' + str(status) + ' [overWriteBool=False]'

		
	def _resetEventFlag(self):
		
		#This should run before timers, eyeTrackingCal.  Called using <vizact>.onupdate
		if( self.lastFrameUpdated == viz.getFrameNumber() ):
			print 'Did not reset! Status already set to ' + str(self.status)
		else:
			self.status = 0; # 0 Means nothing is happening
			self.collisionLocOnObs_XYZ = [nan,nan,nan]

			
			
		
class block():
	def __init__(self,config=None,blockNum=1, room = None):
			
		# Each block will have a block.trialTypeList
		# This list is a list of strings of each trial type
		# Types included and frequency of types are defined in the config
		# Currently, these trial types are automatically randomized
		# e.g. 't1,t2,t2,t2,t1'
		
		self.blockName = config.expCfg['experiment']['blockList'][blockNum]
		self.room = room
	#    Kinds of trial in this block
		
		# trialTypeList enumerates the types of trials
		self.trialTypesInBlock = config.expCfg['blocks'][self.blockName]['trialTypesString'].split(',')
		
		# The number of each type of trial
		self.numOfEachTrialType_type = map(int,config.expCfg['blocks'][self.blockName]['trialTypeCountString'].split(','));
		
		# THe type of each trial
		# _tr indicates that the list is as long as the number of trials
		self.trialTypeList_tr = []
		
		if( len(self.numOfEachTrialType_type)< len(self.trialTypesInBlock) ):
			print 'trialTypeCountString < trialTypesString!  These should be the same length'
			
		for typeIdx in range(len(self.trialTypesInBlock)):
			for count in range(self.numOfEachTrialType_type[typeIdx]):
				self.trialTypeList_tr.append(self.trialTypesInBlock[typeIdx])
		print 'Trial Type List = {', self.trialTypeList_tr,'}'
		# Randomize trial order
		from random import shuffle
		shuffle(self.trialTypeList_tr)
		
		self.numTrials = len(self.trialTypeList_tr)
		self.recalAfterTrial_idx = config.expCfg['blocks'][self.blockName]['recalAfterTrial']
		
		self.trials_tr = []
		
		for trialNumber in range(self.numTrials):
			
			## Get trial info
			trialObj = trial(config,self.trialTypeList_tr[trialNumber], self.room)
				
			##Add the body to the list
			self.trials_tr.append(trialObj)

			## Create a generator this will loop through the balls
			#nextBall = viz.cycle(balls); 
		
class trial(viz.EventClass):
	def __init__(self,config=None,trialType='t1', room = None):
		
		#viz.EventClass.__init__(self)
		self.config = config
		self.trialType = trialType
		self.room = room
		
		## State flags
		self.subIsInBox = False
		self.waitingForGo = False
		self.goSignalGiven = False 		
		self.approachingObs = False
		self.objIsVirtual = int(config.expCfg['trialTypes'][self.trialType]['objIsVirtual'])
		
		self.goSignalTimerObj = []
		self.metronomeTimerObj = []
		self.trialTimeoutTimerObj = []
		
		#self.legLengthCM = config.expCfg['experiment']['legLengthCM']		
		self.obsHeightM = []
		
		# Object placeholders
		self.obsObj = -1
		self.objectSizeText = -1
		
		self.collisionLocGlobal_XYZ = [nan, nan, nan]
		
		self.lineObj = -1
		#self.totalTrialNumber = 
		###########################################################################################
		###########################################################################################
		## Get fixed variables here
			
		try:
			self.obsColor_RGB = map(float,config.expCfg['trialTypes'][self.trialType]['obsColor_RGB'])
		except:
			#print 'Using def color'
			self.obsColor_RGB = map(float,config.expCfg['trialTypes']['default']['obsColor_RGB'])
		
		self.obsHeightLegRatio = float(config.expCfg['trialTypes'][self.trialType]['obsHeightLegRatio'])
		
		self.obsZLoc_distType = []
		self.obsZLoc_distParams = []
		self.obsZLoc = []
		
		self.obsXLoc = config.expCfg['room']['standingBoxOffset_X']
				
		# The rest of variables are set below, by drawing values from distributions
#		
#		Example: ballDiameter and gravity
#		self.ballDiameter_distType = []
#		self.ballDiameter_distParams = []
#		self.ballDiameter = []
#		
#		self.gravity_distType = []
#		self.gravity_distParams = []
#		self.gravity = []
		
		# Go into config file and draw variables from the specified distributions
		# When a distribution is specified, select a value from the distribution
		
		#variablesInATrial = config.expCfg['trialTypes']['default'].keys()
		variablesInATrial = config.expCfg['trialTypes'][self.trialType].keys()
		#print 'All Variables:', variablesInATrial
		# for all of this trial type (e.g. t1, t2...)
		for varIdx in range(len(variablesInATrial)):
			# find the variable with "_distType" in the string/variable name
			if "_distType" in variablesInATrial[varIdx]:
				# Extracts that variable typeobsXLoc_distType
				varName = variablesInATrial[varIdx][0:-9]
				#print '===>Variable Name:', varName
				# _setValueOrUseDefault assigns a value according to the distribution type
				distType, distParams, value = self._setValueOrUseDefault(config,varName)
									
				exec( 'self.' + varName + '_distType = distType' )
				exec( 'self.' + varName + '_distParams = distParams' )
				exec( 'self.' + varName + '_distType = distType' )
				# Draw value from a distribution
				exec( 'self.' + varName + ' = drawNumberFromDist( distType , distParams);' )

#	def removeBall(self):
#		An example of how to remove an object from the room
#		self.ballObj.remove()
#		self.ballObj = -1
#		
#		self.ballInRoom = False
#		self.ballInInitialState = False
#		self.ballLaunched = False
		

	def _setValueOrUseDefault(self,config,paramPrefix):
		
		try:
			# Try to values from the subsection [[trialType]]
			distType = config.expCfg['trialTypes'][self.trialType][paramPrefix + '_distType']
			distParams = config.expCfg['trialTypes'][self.trialType][paramPrefix +'_distParams']
			#print 'Using Config File for Parameters =>', paramPrefix
			
		except:
			# Try to values from the subsection [['default']]
			distType = config.expCfg['trialTypes']['default'][paramPrefix + '_distType'];
			distParams = config.expCfg['trialTypes']['default'][paramPrefix + '_distParams'];
			print 'Using default: **' + paramPrefix + '**'
		
		
		#print 'Distribution :', distType, distParams
		value = drawNumberFromDist(distType,distParams)
	
		
		return distType,distParams,value
		
			
	def placeObs(self,room):
		#experimentObject.currentTrial.obsObj.collisionBox.physNode.body.getPosition()
		
		#print 'Creating object at ' + str(obsLoc)
		
		self.obsHeightM = self.config.legLengthCM * self.obsHeightLegRatio / 100
		#print str(self.obsHeightLegRatio) + ' + ' + str(self.config.legLengthCM) + ' = ' + str(self.obsHeightM)
		
		#obsSize = [0.015,1.2,self.obsHeightM] # lwh
		obsLoc = [self.obsXLoc,0,self.obsZLoc]
		
		import obstacleClass
		#self.obsObj = visEnv.visObj(room,'box',obsSize,obsLoc,self.obsColor_RGB)
		#self.obsObj.enablePhysNode()
		
		if( self.objIsVirtual == False ):

			self.obsObj = visEnv.visObj(room,'box',obsSize,obsLoc,self.obsColor_RGB)
			
			self.obsObj.visNode.disable(viz.RENDERING)
			
			# Draw a line on the ground
			lineHeight = 0.001
			lineSize = [0.015,5.0,lineHeight] # lwh
			obsLoc = [self.obsXLoc,lineHeight/2,self.obsZLoc]
			self.lineObj = visEnv.visObj(room,'box',lineSize,obsLoc,self.obsColor_RGB)
			
			if( self.trialType == 't4' ):
				displayText = 'Short'
				#print 'Obs Height =>', displayText, self.trialType
			elif( self.trialType == 't5' ):
				displayText = 'Med'
				#print 'Obs Height =>', displayText, self.trialType
			elif( self.trialType == 't6' ):
				displayText = 'Tall'
				#print 'Obs Height =>', displayText, self.trialType
			else:
				print 'Object height invalid! Trial Type :', self.trialType
				displayText = 'Unknown!!'
				return
				
			self.objectSizeText = viz.addText3D(displayText)
			self.objectSizeText.setEuler([-90,90,0], viz.ABS_GLOBAL)
			self.objectSizeText.setPosition([-1.,.001,1.3],viz.ABS_GLOBAL)
			scale = 0.1
			self.objectSizeText.setScale([scale ,scale ,scale])
		
		else:
			
			self.obsObj = obstacleObj(room,self.obsHeightM,obsLoc)
			
			print'Placing Obstacle at', obsLoc, 'height', self.obsHeightM

	def removeObs(self):

		if( self.objectSizeText != -1 and (self.trialType == 't4' or self.trialType == 't5'  or self.trialType == 't6' )):
			self.objectSizeText.remove()
			self.objectSizeText = -1
			
		if( self.obsObj != -1):
			self.obsObj.remove()		
			self.obsObj = -1
		
		if( self.lineObj != -1):
			self.lineObj.remove()		
			self.lineObj = -1
	
def demoMode(experimentObject):
	
#	duckBeginPos =  experimentObject.config.virtualPlane.getCenterPos('floor')
#	duck = viz.addAvatar('duck.cfg',pos=duckBeginPos)
#	duck.scale([0.5,0.5,0.5])
	
	##Add a world axis with X,Y,Z labels
#	world_axes = vizshape.addAxes(.3) 
#	X = viz.addText3D('X',pos=[0.33,0,0],color=viz.RED,scale=[0.1,0.1,0.1],parent=world_axes)
#	Y = viz.addText3D('Y',pos=[0,0.33,0],color=viz.GREEN,scale=[0.1,0.1,0.1],align=viz.ALIGN_CENTER_BASE,parent=world_axes)
#	Z = viz.addText3D('Z',pos=[0,0,0.33],color=viz.BLUE,scale=[0.1,0.1,0.1],align=viz.ALIGN_CENTER_BASE,parent=world_axes)
	
	experimentObject.room.standingBox.remove()
	experimentObject.room.floor.visNode.remove()
	
	viz.killtimer(experimentObject.perFrameTimerID)
	
	#vizshape.addGrid()
	
	global piazza
	piazza = viz.add('piazza.osgb')
	piazza.setScale([.15,.15,.15])
	piazza.setPosition([0,0,-3])
	
	
################################################################################################################   
################################################################################################################
################################################################################################################
##  Here's where the magic happens!

## vrlabConfig uses config to setup hardware, motion tracking, frustum, eyeTrackingCal.
##  This draws upon the system config to setup the hardware / HMD

## The experiment class initialization draws the room, sets up physics, 
## and populates itself with a list of blocks.  Each block contains a list of trials

experimentObject = Experiment(expConfigFileName)
experimentObject.start()
#
demoMode(experimentObject)
#grid = vizshape.addGrid()
#grid.scale([0.25,0.25,0.25])

# If you want to see spheres for each marker
#visEnv.drawMarkerSpheres(experimentObject.room,experimentObject.config.mocap)


#vizshape.addBox(size=(0.05,0.05,0.05))
if( experimentObject.hmdLinkedToView == False ):
	
	#print 'Head controlled by mouse/keyboard. Initial viewpoint set in vrLabConfig _setupSystem()'
	
	#viz.MainView.setPosition(-3,2,-3)
	#viz.MainView.setPosition([experimentObject.room.wallPos_NegX +.1, 2, experimentObject.room.wallPos_NegZ +.1])
	#viz.MainView.lookAt([0,2,-2])
	# Setup keyboard/mouse trackerklkk
	
	import vizcam
	
	#tracker = vizcam.addKeyboard6DOF(moveScale=1.0)
	#tracker.setPosition([-3,.5,0])
	#tracker.lookAt([3,0.5,0])
	
	#viz.link(tracker,viz.MainView)
	#viz.mouse.setVisible(False)
	
	
vizact.onkeydown('b',experimentObject.config.mocap.getRigidMarkerPositions,'right')