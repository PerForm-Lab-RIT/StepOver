"""
Runs an experiment.

Things required for transition to PandA Lab setup:

- rename performLabVR2.cfg as <machineName>.cfg
- replace all mocap calls with those particular to your system.  I tried to mark them with MOCAP
- buy a wiimote, or goto <experiment>.registerWiimoteActions and associate the callbacks with keyboard presses
- 

90
"""

viz.res.addPath('resources')
sys.path.append('utils')

import viz
import viztask
import vizshape
import vizact
from drawNumberFromDist import *
import ode
import datetime
from ctypes import * # eyetrackka
import winsound
import virtualPlane
import vizinput
import time

#import visEnv12
#import physEnv

import visEnv
import physEnv

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


#############################
useNetwork = True

if( useNetwork ):
	networkClient = 'performVR'.upper()
	viz.net.addPort(5000) # Send data over port 5000

def appendTrialToEndOfBlock(e):
	
	experimentObject.appendTrialToEndOfBlock()
	
	import winsound
	Freq = 1500 # Set Frequency To 2500 Hertz
	Dur = 100 # Set Duration To 1000 ms == 1 second
	winsound.Beep(Freq,Dur)
	
def onNetwork(netPacket):
	
	print 'Received network message'

	if netPacket.sender.upper() == networkClient:
		experimentObject.eventFlag.setStatus(8)
		netPacket.action(netPacket)

viz.callback(viz.NETWORK_EVENT, onNetwork)

#############################		

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
		self.beep_f = '/Resourcs/beep_Spike.wav'
		
		viz.playSound(self.gong,viz.SOUND_PRELOAD)
		viz.playSound(self.beep,viz.SOUND_PRELOAD)
		viz.playSound(self.bounce,viz.SOUND_PRELOAD)
		viz.playSound(self.buzzer,viz.SOUND_PRELOAD)
		viz.playSound(self.bubblePop,viz.SOUND_PRELOAD)
		viz.playSound(self.highDrip,viz.SOUND_PRELOAD)
		viz.playSound(self.cowbell,viz.SOUND_PRELOAD)
		viz.playSound(self.beep,viz.SOUND_PRELOAD)

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
			
		self.vizconnect = vizconnect.go( './vizConnect/' + self.sysCfg['vizconfigFileName'])
		self.__postVizConnectSetup()
		
	def __postVizConnectSetup(self):
		
		''' 
		This is where one can run any system-specifiRRRc code that vizconnect can't handle
		'''

		dispDict = vizconnect.getRawDisplayDict()
				
		if self.sysCfg['use_phasespace']:
			
			from mocapInterface import phasespaceInterface	
			self.mocap = phasespaceInterface(self.sysCfg);
			self.mocap.start_thread()
			
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
		
		self.__setWinPriority()
		viz.setMultiSample(self.sysCfg['antiAliasPasses'])
		viz.MainWindow.clip(0.01 ,200)
		
		viz.vsync(1)
		viz.setOption("viz.glfinish", 1)
		viz.setOption("viz.dwm_composition", 0)
		
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

class Experiment(viz.EventClass):
	
	"""
	Experiment manages the basic operation of the experiment.
	"""
	
	def __init__(self, expConfigFileName):
		
		# Event class
		# This makes it possible to register callback functions (e.g. activated by a timer event)
		# within the class (that accept the implied self argument)
		# eg self.callbackFunction(arg1) would receive args (self,arg1)
		# If this were not an eventclass, the self arg would not be passed = badness.
		
		viz.EventClass.__init__(self)
		
		##############################################################
		##############################################################
		## Use config to setup hardware, motion tracking, frustum, eyeTrackingCal.
		##  This draws upon the system config to setup the hardware / HMD
		
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

		self.directionArrow = vizshape.addArrow(color=viz.BLUE, axis = vizshape.AXIS_X, length=0.2,radiusRatio=0.05 )
		self.directionArrow.setEuler([270,0,0])
		self.directionArrow.setPosition([-1.1,0,-1.5])
		
		################################################################
		################################################################
		# Build block and trial list
		
		self.blockNumber = 0;
		self.trialNumber = 0;
		self.expInProgress = True;
		
		self.totalTrialNumber = 0;
		self.blocks_bl = []
		
		self.room.offsetDistance = float(config.expCfg['room']['minObstacleDistance'])
		
		#print'====> Obstacle Distance = ', self.room.offsetDistance
		for bIdx in range(len(config.expCfg['experiment']['blockList'])):
			self.blocks_bl.append(block(config,bIdx, self.room));
		
		
		self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
		
		################################################################
		################################################################
		##  Misc. Design specific items here.
		
		self.maxTrialDuration = config.expCfg['experiment']['maxTrialDuration']
		
		
		if( config.wiimote ):
			self.registerWiimoteActions()

		#self.obstacleViewTimerID = viz.getEventID('obstacleViewTimerID') # Generates a unique ID. 
		
		self.numClicksBeforeGo = config.expCfg['experiment']['numClicksBeforeGo']
		self.trialEndPosition = config.expCfg['experiment']['trialEndPosition']
		self.metronomeTimeMS = config.expCfg['experiment']['metronomeTimeMS']
				
		##  Setup virtual plane
		if( self.config.use_phasespace == True and self.config.sysCfg['virtualPlane']['attachGlassesToRigid']):
		
			#eyeSphere = visEnv.visObj(self.room,'sphere',size=0.1,alpha=1)
			#eyeSphere.node3D.setParent(self.room.objects)
			
			self.setupShutterGlasses()
			self.setupFeet()
			pass
			
		##############################################################
		##############################################################
		## Callbacks and timers
		
		vizact.onupdate(viz.PRIORITY_PHYSICS,self._checkForCollisions)
		
		self.callback(viz.KEYDOWN_EVENT,  self.onKeyDown)
		self.callback(viz.KEYUP_EVENT, self.onKeyUp)
		self.callback( viz.TIMER_EVENT,self._timerCallback )
		
		self.perFrameTimerID = viz.getEventID('perFrameTimerID') # Generates a unique ID.
		self.starttimer( self.perFrameTimerID, viz.FASTEST_EXPIRATION, viz.FOREVER)
		
		self.trialTimeoutTimerID = viz.getEventID('trialTimeoutTimerID') # Generates a unique ID.
	
		##############################################################
		## Data output
		
		now = datetime.datetime.now()
		dateTimeStr = str(now.year) + '-' + str(now.month) + '-' + str(now.day) + '-' + str(now.hour) + '-' + str(now.minute)
		 
		import os
		
		dataOutPutDir = config.sysCfg['writer']['outFileDir'] + '//' +str(dateTimeStr) + '//'
		#dataOutPutDir = config.sysCfg['writer']['outFileDir'] +str(dateTimeStr) 
		
		if not os.path.exists(dataOutPutDir):
			os.makedirs(dataOutPutDir)
		
		# Exp data file
		self.expDataFile = open(dataOutPutDir + 'exp_data-' + dateTimeStr + '.txt','w+')
		# Function to automate exp data writeout
		self.writeOutDataFun = vizact.onupdate(viz.PRIORITY_LAST_UPDATE,self.writeDataToText)
		
		# Write experiment metadata out to line 1 of exp data file12
		
		expMetaDataStr = ''
		expMetaDataStr = self.getExperimentMetaData()
		self.expDataFile.write(expMetaDataStr + '\n')
		
		# MocapInterface handles writing mocap data in a seperate thread. 
		#self.config.mocap.startLogging('F:\Data\Stepover')
		self.config.mocap.createLog(dataOutPutDir)
		
		from shutil import copyfile
		
		# Copy config files
		copyfile('.\\' + expConfigFileName, dataOutPutDir+expConfigFileName ) # exp config
		copyfile('.\\expCfgSpec.ini', dataOutPutDir + 'expCfgSpec.ini' )# exp config spec1
		
		copyfile('.\\'+os.environ['COMPUTERNAME'] + '.cfg', dataOutPutDir+os.environ['COMPUTERNAME'] + '.cfg')# system config 
		copyfile('.\\sysCfgSpec.ini', dataOutPutDir+ 'sysCfgSpec.ini') # system config spec
		
		##############################################################
		## Event flag
		
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
	
					self.currentTrial.goSignalTimerObj.setEnabled(viz.TOGGLE);
					self.currentTrial.goSignalTimerObj = [];
				
				##############################################
				### Go signal already given.  Starting the trial
				elif(self.currentTrial.goSignalGiven is True):
					
					print 'Starting trial ==> Type', self.currentTrial.trialType
					self.eventFlag.setStatus(1)
					self.currentTrial.approachingObs = True
					self.currentTrial.startTime = time.time()
					
					# Start data collection
					viz.playSound(soundBank.beep_f)
					
					if( type(self.currentTrial.goSignalTimerObj) is not list ):			
						self.currentTrial.goSignalTimerObj.remove()
						
					
					vizact.ontimer2(self.maxTrialDuration, 0,self.endTrial)
					
										
					#self.maxTrialDurationObj
					#self.trialTimeoutTimerID
				
	def _checkForCollisions(self):
		
		thePhysEnv = self.room.physEnv;
		
		if( self.eventFlag.status == 6 or # A bit of messy schedulign.  Trial ending, so collision box does not exist.
			self.eventFlag.status == 7 or
			thePhysEnv.collisionDetected == False or 
			self.expInProgress == False ): 
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
					
					collisionLoc_XYZ,normal,depth,geom1,geom2 = thePhysEnv.contactObjects_idx[0].getContactGeomParams()
					self.currentTrial.collisionLocOnObs_XYZ = collisionLoc_XYZ
					
					viz.playSound(soundBank.beep)
				
				elif( physNode1 == rightFoot.physNode and physNode2 == obstacle.physNode ):
					
					self.eventFlag.setStatus(5)
					collisionLoc_XYZ,normal,depth,geom1,geom2 = thePhysEnv.contactObjects_idx[0].getContactGeomParams()

					self.currentTrial.collisionLocOnObs_XYZ = collisionLoc_XYZ
										
					viz.playSound(soundBank.beep)
			
	def start(self):
		
		##This is called when the experiment should begin.
		self.setEnabled(True)

	
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
		
		if (self.inCalibrateMode is False):

			##  More MocapInterace functions
			if key == 'P':
				mocapSys.resetRigid('spine') 
			elif key == 'S':
				mocapSys.resetRigid('shutter')
			elif key == 'L':
				mocapSys.resetRigid('left') 
				self.resizeFootBox('left')
				
			elif key == 'R':
				mocapSys.resetRigid('right')
				self.resizeFootBox('right')
			
			elif key == 'O':
				experimentObject.currentTrial.removeObs()
				
				
			if( viz.key.isDown( viz.KEY_CONTROL_L )):
				
				##  More MocapInterace functions
				if key == 'p':
					mocapSys.saveRigid('spine') # MOCAP
				elif key == 's':
					mocapSys.saveRigid('shutter') # MOCAP
				elif key == 'l':
					mocapSys.saveRigid('left') # MOCAP
				elif key == 'r':
					mocapSys.saveRigid('right') # MOCAP
				
	def onKeyUp(self,key):
		
		if( key == 'v'):
			pass
			
	def getExperimentMetaData(self):
		'''  
		Write out experiment specific metadeta
		MocapInterace - some parmaeters are specific to the mocap system
		'''
		config = self.config
		outputString = '';
		
		outputString = outputString  + 'legLengthCM %f ' % (config.legLengthCM)
		
		outputString = outputString  + 'maxTrialDuration %f ' % (self.config.expCfg['experiment']['maxTrialDuration'])
	
		## Numblocks and block list
		numBlocks = len(config.expCfg['experiment']['blockList']);
		outputString = outputString  + 'numBlocks %f ' % (numBlocks)
						
		outputString = outputString  + 'mocapRefresh %f ' % (config.mocap.owlParamFrequ)
		outputString = outputString  + 'mocapInterp %f ' % (config.mocap.owlParamInterp)
		outputString = outputString  + 'mocapPostProcess %f ' % (config.mocap.owlParamPostProcess)
		
		
		return outputString 
		
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
		outputString = '';
		outputString = outputString + 'frameTime %f ' % (viz.getFrameTime())
		outputString = outputString + 'sysTime %f ' % (time.time())
		
		outputString = outputString + ' eventFlag %d ' % (self.eventFlag.status)
			
		
		if( self.eventFlag.status == 1 ):
			
			outputString = outputString + ' trialType %s ' % (self.currentTrial.trialType)
			
			## =======================================================================================================
			## Obstacle Height & Location 
			## =======================================================================================================
			outputString = outputString + '[ obstacle_XYZ %f %f %f ] ' % (self.currentTrial.obsLoc_XYZ[0],self.currentTrial.obsLoc_XYZ[1],self.currentTrial.obsLoc_XYZ[2])
			
			outputString = outputString + ' obstacleHeight %f ' % (self.currentTrial.obsHeightM)
			
			outputString = outputString + ' isWalkingDownAxis %d ' % (self.room.isWalkingDownAxis)
			outputString = outputString + ' trialNum %d ' % (self.trialNumber)
			
			outputString = outputString + ' standingBoxOffset_negZ %d ' % (self.room.standingBoxOffset_negZ)
			outputString = outputString + ' standingBoxOffset_posZ %d ' % (self.room.standingBoxOffset_posZ)
			
			leftFoot_LWH = self.room.leftFoot.node3D.getBoundingBox().getSize()
			outputString = outputString + ' [ leftFoot_LWH %f %f %f ] ' % (leftFoot_LWH[2], leftFoot_LWH[0], leftFoot_LWH[1])
			
			rightFoot_LWH = self.room.rightFoot.node3D.getBoundingBox().getSize()
			outputString = outputString + ' [ rightFoot_LWH %f %f %f ] ' % (rightFoot_LWH[2], rightFoot_LWH[0], rightFoot_LWH[1])
			
			
		if( self.eventFlag.status == 4 or self.eventFlag.status == 5 ):		

			collisionPosLocal_XYZ = self.currentTrial.collisionLocOnObs_XYZ
			outputString = outputString + '[ collisionLocOnObs_XYZ %f %f %f ] ' % (collisionPosLocal_XYZ[0], collisionPosLocal_XYZ[1], collisionPosLocal_XYZ[2])
		
		## =======================================================================================================
		## VisNode body positions and quaternions
		## =======================================================================================================
		
		##################################################
		# Right foot
		
		rightFootPos_XYZ = []
		rightFootQUAT_XYZW = []
		
		if( self.room.rightFoot ):
			
			rightFootPos_XYZ = self.room.rightFoot.node3D.getPosition()
			rightFootMat = self.room.rightFoot.node3D.getMatrix()
			rightFootQUAT_XYZW = rightFootMat.getQuat()
			
		else:
			rightFootPos_XYZ = [None, None, None]
			rightFootQUAT_XYZW = [None, None, None]
		
		outputString = outputString + '[ rFoot_XYZ %f %f %f ] ' % (rightFootPos_XYZ[0], rightFootPos_XYZ[1], rightFootPos_XYZ[2])
		outputString = outputString + '[ rFootQUAT_XYZW %f %f %f %f ] ' % ( rightFootQUAT_XYZW[0], rightFootQUAT_XYZW[1], rightFootQUAT_XYZW[2], rightFootQUAT_XYZW[3] )
		
		##########
		# Left foot
		leftFootPos_XYZ = []
		leftFootQUAT_XYZW = []
		
		if( self.room.leftFoot ):
			
			leftFootPos_XYZ = self.room.leftFoot.node3D.getPosition()
			leftFootMat = self.room.leftFoot.node3D.getMatrix()
			leftFootQUAT_XYZW = leftFootMat.getQuat()
			
		else:
			leftFootPos_XYZ = [None, None, None]
			leftFootQUAT_XYZW = [None, None, None]
			
		outputString = outputString + '[ lFoot_XYZ %f %f %f ] ' % (leftFootPos_XYZ[0], leftFootPos_XYZ[1], leftFootPos_XYZ[2])
		outputString = outputString + '[ lFootQUAT_XYZW %f %f %f %f ] ' % ( leftFootQUAT_XYZW[0], leftFootQUAT_XYZW[1], leftFootQUAT_XYZW[2], leftFootQUAT_XYZW[3] )
		
		##########
		# Glasses 
		
		glassesPos_XYZ = []
		glassesQUAT_XYZW = []
		
		if( self.room.eyeSphere ):
			
			glassesPos_XYZ = self.room.eyeSphere.node3D.getPosition()
			glasses = self.room.eyeSphere.node3D.getMatrix()
			glassesQUAT_XYZW = glasses.getQuat()
			
		else:
			glassesPos_XYZ = [None, None, None]
			glassesQUAT_XYZW = [None, None, None]
			
		outputString = outputString + '[ glasses_XYZ %f %f %f ] ' % (glassesPos_XYZ[0], glassesPos_XYZ[1], glassesPos_XYZ[2])
		outputString = outputString + '[ glassesQUAT_XYZW %f %f %f %f ] ' % ( glassesQUAT_XYZW[0], glassesQUAT_XYZW[1], glassesQUAT_XYZW[2], glassesQUAT_XYZW[3] )
		
		##########
		# Mainview
		
		viewPos_XYZ = viz.MainView.getPosition()
		outputString = outputString + '[ viewPos_XYZ %f %f %f ] ' % (viewPos_XYZ[0],viewPos_XYZ[1],viewPos_XYZ[2])

		viewQUAT_XYZW = viz.MainView.getMatrix()
		outputString = outputString + '[ viewQUAT_XYZW %f %f %f %f ] ' % ( viewQUAT_XYZW[0], viewQUAT_XYZW[1], viewQUAT_XYZW[2], viewQUAT_XYZW[3] )
		
		################################################################################################
		## Record rigid body pos / quat, and marker on rigid pos 
		################################################################################################
		
		if( self.eventFlag.status == 6 or self.eventFlag.status == 7 ):

			mocap = self.config.mocap
			#trialDuration = time.time() - self.currentTrial.startTime
		
		return outputString #%f %d' % (viz.getFrameTime(), self.inCalibrateMode)
		
	def toggleWalkingDirection(self):
		'''
		Relocates the standing box / box in which the subject stands to see the obstacle
		'''
		
		print 'Changing Direction From ' + str(self.room.isWalkingDownAxis)+' to ' + str(not(self.room.isWalkingDownAxis))
		
		# Flip walking direction indicator
		self.room.isWalkingDownAxis = not(self.room.isWalkingDownAxis)
		self.standingBoxOffsetX = self.room.standingBoxOffset_X
		
		
		if( self.room.isWalkingDownAxis ):
			
			self.directionArrow.setEuler([90,0,0])			
			standingBoxZPos = self.room.standingBoxOffset_posZ
			self.room.standingBox.setPosition([self.standingBoxOffsetX, 0.1, standingBoxZPos])
			
		else:
			self.directionArrow.setEuler([270,0,0])
			standingBoxZPos = self.room.standingBoxOffset_negZ
			self.room.standingBox.setPosition([self.standingBoxOffsetX, 0.1, standingBoxZPos])

	def endTrial(self):
		
		self.eventFlag.setStatus(6,True)
		
		endOfTrialList = len(self.blocks_bl[self.blockNumber].trials_tr)
		
		self.toggleWalkingDirection();	
		
		#self.currentTrial.approachingObs = False
		
		#print 'Ending block: ' + str(self.blockNumber) + 'trial: ' + str(self.trialNumber)
		
		# If it is the last trial...
		if( self.trialNumber < endOfTrialList ):
			
			recalAfterTrial_idx = self.blocks_bl[self.blockNumber].recalAfterTrial_idx
			
			if( recalAfterTrial_idx.count(self.trialNumber ) > 0):
				soundBank.gong.play()
				vizact.ontimer2(0,0,self.toggleEyeCalib)
				
			# Increment trial 
			self.trialNumber += 1
			
			## Play sound
			viz.playSound(soundBank.beep_f)
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
			
			# Stop loggging mocap data
			self.config.mocap.stopLogging()
			
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
			
			vizact.ontimer2(0,0,self.nextTrial)

			return
			
			
	def writeDataToText(self):
		

		# Only write data if the experiment is ongoing and sub is approaching obstacle
		if( self.expInProgress and self.currentTrial.approachingObs ):
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
			
			# MOCAP - self.config.virtualPlane.setNewCornerPosition uses sensed marker position
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_LEFT,env.setNewCornerPosition,0,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_UP,env.setNewCornerPosition,1,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_RIGHT,env.setNewCornerPosition,2,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_DOWN,env.setNewCornerPosition,3,markerNum)
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_PLUS,env.updatePowerwall)

			vizact.onsensorup(self.config.wiimote,wii.BUTTON_MINUS,viz.MainWindow.setStereoSwap,viz.TOGGLE)

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
		
		# Important because it calls the join() command
		# <thread>.join() merges the thread and prevents a premature exit
		# ... for exaple, before the thread has finished writing mocap data to the file
		self.config.mocap.stop_thread()
		
		print 'End of Trial & Block ==> TxT file Saved & Closed'
		print 'end experiment'
		self.expInProgress = False
		viz.playSound(soundBank.gong)

	def giveGoSignal(self):
		print 'Go signal given!'
		self.currentTrial.goSignalGiven = True
		
		if( type(self.currentTrial.metronomeTimerObj) is not list ):			
			self.currentTrial.metronomeTimerObj.remove()

		viz.playSound(soundBank.beep)
		
		# Change the Obstacle Color to Green as a visual feedback for the subject
		if( self.currentTrial.objIsVirtual):
			self.currentTrial.obsObj.setColor(viz.WHITE)
		
		# Start logging data
		self.config.mocap.writeStringToLog('Trial: ' + str(self.trialNumber+1))
		self.config.mocap.startLogging()
		
		
	def inputLegLength(self):
		
		#print('SETTING LEG LENGTH TO 100!')
		#self.config.obsHeightLegRatio = 100

		#prompt for an integer value of leg length in CM
		self.config.legLengthCM = float(vizinput.input('Enter leg length (cm):', value = float(90)))
		
		try:
			# Test if it's an integer
			intValue = int(self.config.legLengthCM)
		except ValueError:
			viz.message( 'Please enter a valid integer')
			self.inputLegLength()
	
	def isVisObjInBox(self,visObj):
		'''
		Check to see if the visObj is currently inside the standing box
		This fun is run on every frame for both feet prior before the go signal is given
		'''
		
		objPos_xyz = visObj.node3D.getPosition()
		
		boxPos_xyz = self.room.standingBox.getPosition();
		
		standingBoxOffsetX = boxPos_xyz[0]
		standingBoxOffsetZ = boxPos_xyz[2]
		standingBoxSize_WHL = self.config.expCfg['room']['standingBoxSize_WHL']
		
			
		# Is the head inside the standing box?
		if( objPos_xyz[0] > (standingBoxOffsetX - standingBoxSize_WHL[0]/2) and 
			objPos_xyz[0] < (standingBoxOffsetX + standingBoxSize_WHL[0]/2) and
			objPos_xyz[2] > (standingBoxOffsetZ - standingBoxSize_WHL[2]/2) and 
			objPos_xyz[2] < (standingBoxOffsetZ + standingBoxSize_WHL[2]/2)):
		
			return 1
		else:
			return 0
	
	def setupShutterGlasses(self):

		'''
		MOCAP: This is where I create an action that updates the mainview
		with data from my motion capture system
		'''
		
		config = self.config
		print 'Connecting mainview to eyesphere'

		# Flip L/R eye phasing
		viz.MainWindow.setStereoSwap(viz.TOGGLE)
		
		eyeSphere = self.room.eyeSphere
		eyeSphere.node3D.visible(viz.TOGGLE)
		
		shutterRigid = config.mocap.returnPointerToRigid('shutter') # Here, 
		shutterRigid.link_pose(eyeSphere.node3D)
				
		self.config.virtualPlane.attachViewToGlasses(eyeSphere.node3D,shutterRigid)
			
	def setupFeet(self):

		'''
		MOCAP: This is where I create an action that updates the tracked foot position
		with data from the motion capture device
		'''
			
		config = self.config
		leftFoot = self.room.leftFoot
		leftFoot.node3D.color([0, 0, .3])
		
		# First, link visual to rigid body
		lFootRigid = config.mocap.returnPointerToRigid('leftFoot')
		lFootRigid.link_pose(leftFoot.node3D)
		
		# Now, link physical to visal
		leftFoot.enablePhysNode()
		leftFoot.physNode.isLinked = 1
		viz.link( leftFoot.node3D, leftFoot.physNode.node3D, priority = viz.PRIORITY_LINKS+1)
		
		rightFoot = self.room.rightFoot
		rightFoot.node3D.color([0.5, 0, 0])
		rFootRigid = config.mocap.returnPointerToRigid('rightFoot')
		rFootRigid.link_pose(rightFoot.node3D)
		rightFoot.enablePhysNode()
		rightFoot.physNode.isLinked = 1
		viz.link( rightFoot.node3D, rightFoot.physNode.node3D,priority = viz.PRIORITY_LINKS+1)
		
	def resizeFootBox(self,footSide):
		''' 
		MocapInterace:  use foot markers to resize the rigid body to the dimensions of the foot
		
		Algorithm is:
		
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
	
		footRigid = mocap.returnPointerToRigid(footSide)
	
		# Get positions of markers on rigid body in world coordinates
		markerDict = footRigid.get_markers() # MocapInterace
		
		# Get marker positions
		markerPosViz_mIdx_XYZ = [markerDict[mKey].pos for mKey in markerDict.keys()]
		
		# Get X / Z positions
		markerXVals_mIdx = [markerPosViz_mIdx_XYZ[mIdx][0] for mIdx in range(len(markerPosViz_mIdx_XYZ))]
		markerZVals_mIdx = [markerPosViz_mIdx_XYZ[mIdx][2] for mIdx in range(len(markerPosViz_mIdx_XYZ))]

		if( markerPosViz_mIdx_XYZ == -1 ): # or len(markerPosViz_mIdx_XYZ) < expected number
			print 'Error: Could not see all foot markers'
			return
		
		# Set length and width
		footWidth = max(markerXVals_mIdx) - min(markerXVals_mIdx)
		footLength = (max(markerZVals_mIdx) - min(markerZVals_mIdx))
		
		# Take average height of center markers
		sumOfMarkerHeights = 0
		
		for mIdx in footRigid.center_marker_ids:
			sumOfMarkerHeights = sumOfMarkerHeights + markerPosViz_mIdx_XYZ[mIdx][1]
			
		footHeight = sumOfMarkerHeights / len(footRigid.center_marker_ids)
		
		footLWH = [footLength, footWidth, footHeight]
		
		if( footSide == 'left' ):			
			footObj = self.room.leftFoot
		
		elif( footSide == 'right' ):
			footObj= self.room.rightFoot
		
		footObj.size = footLWH
		
		print 'Making basic node3D of size ' + str(footLWH)

		#### Redraw rigid bodies with new dimensions
		
		# Erase vis and phys component of foot 
		footObj.removePhysNode()
		footObj.node3D.remove()
		
		# ...rebuild vis object according to new dimensions in footObj.size
		footObj.makeBasicVizShape()
		
		# ...mow, link to rigid bodies and turn on physNodes
		self.setupFeet()
	
	def nextTrial(self):
	
		print 'Starting block: ' + str(self.blockNumber) + ' Trial: ' + str(self.trialNumber)
		self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
	


	def appendTrialToEndOfBlock(self):
		
		print 'Appending current trial onto the end of the block list.'
		self.blocks_bl[self.blockNumber].trials_tr.append(self.currentTrial)
		
		
class eventFlag(viz.EventClass):
	
	def __init__(self):
		
		################################################################
		##  Eventflag
		
		# 1 Trial Start
		# 2 Experiment start
		# 3 
		# 4 Right foot collides with obstacle
		# 5 Left foot collides with obstacle
		# 6 Trial end
		# 7 Block end
		# 8 Network signalled trial redo
		
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
		
		self.startTime = []
		
		#viz.EventClass.__init__(self)
		self.config = config
		self.trialType = trialType
		self.room = room
		self.obsLoc_XYZ = []
		self.collisionLocOnObs_XYZ = [-1,-1,-1]
		
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
		
		###########################################################################################
		###########################################################################################
		## Get fixed variables here
			
		try:
			self.obsColor_RGB = map(float,config.expCfg['trialTypes'][self.trialType]['obsColor_RGB'])
		except:
			#print 'Using def color'
			self.obsColor_RGB = map(float,config.expCfg['trialTypes']['default']['obsColor_RGB'])
		
		self.obsHeightLegRatio = float(config.expCfg['trialTypes'][self.trialType]['obsHeightLegRatio'])
		
		self.obsDistance_distType = []
		self.obsDistance_distParams = []
		self.obsDistance = []
		
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
				#print '===> Variable Name:', varName
				# _setValueOrUseDefault assigns a value according to the distribution type
				distType, distParams, value = self._setValueOrUseDefault(config,varName)
									
				exec( 'self.' + varName + '_distType = distType' )
				exec( 'self.' + varName + '_distParams = distParams' )
				exec( 'self.' + varName + '_distType = distType' )
				# Draw value from a distribution
				exec( 'self.' + varName + ' = drawNumberFromDist( distType , distParams);' )

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
		
		self.obsZPos = []
		if( self.room.isWalkingDownAxis ):

			self.obsZPos = self.room.standingBoxOffset_posZ - self.obsDistance
			
		else:
			self.obsZPos = self.room.standingBoxOffset_negZ + self.obsDistance
			
		
		self.obsLoc_XYZ = [self.room.standingBoxOffset_X,0,self.obsZPos]
				
		if( self.objIsVirtual == False ):
			
			
			# I place a virtual obstacle but dont render it
			# This allows for collision detection
			# also, the same analysis can be applied to real-world and virtual trials
			self.obsObj = obstacleObj(room,self.obsHeightM,self.obsLoc_XYZ)			
			self.obsObj.node3D.disable(viz.RENDERING)
			
			#experimentObject.currentTrial.obsObj.crossBarHeight
			
			# Draw a line on the ground
			lineHeight = 0.001
			lineSize = [0.015,5.0,lineHeight] # lwh
			
			lineLoc = [self.room.standingBoxOffset_X,lineHeight/2,self.obsZPos]

			self.lineObj = visEnv.visObj(room,'box',lineSize,lineLoc,self.obsColor_RGB)
			
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
			
			self.obsObj = obstacleObj(room,self.obsHeightM,self.obsLoc_XYZ)
			self.obsObj.node3D.color(viz.RED)
			print'Placing Obstacle at', self.obsLoc_XYZ, 'height', self.obsHeightM
	
		
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

	experimentObject.room.leftFoot.remove()
	experimentObject.room.rightFoot.remove()
	experimentObject.room.standingBox.remove()
	experimentObject.room.floor.node3D.remove()
	
	experimentObject.directionArrow.remove()
	
	experimentObject.currentTrial.removeObs()
	
	#viz.killtimer(experimentObject.perFrameTimerID)
	
	#vizshape.addGrid()
	
	global piazza
	piazza = viz.add('piazza.osgb')
	piazza.setScale([.15,.15,.15])
	piazza.setPosition([0,0,-3])
	
	
################################################################################################################   
###################################################################90
#############################################
################################################################################################################
##  Here's where the magic happens!

## vrlabConfig uses config to setup hardware, motion tracking, frustum, eyeTrackingCal.
##  This draws upon the system config to setup the hardware / HMD

## The experiment class initialization draws the room, sets up physics, 
## and populates itself with a list of blocks.  Each block contains a list of trials

experimentObject = Experiment(expConfigFileName)
experimentObject.start()

lf = experimentObject.room.leftFoot
#lf.node3D.disable(viz.RENDERING)

rf = experimentObject.room.rightFoot
#rf.node3D.disable(viz.RENDERING)

#lr = experimentObject.config.mocap.returnPointerToRigid('left')

#lf.node3D.getPosition()
#lr.get_position()

#demoMode(experimentObject)

#grid = vizshape.addGrid()
#grid.scale([0.25,0.25,0.25])

# If you want to see spheres for each marker

def checkObs():
	print experimentObject.currentTrial.obsObj.getPosition()
	print experimentObject.currentTrial.obsObj.collisionBox.node3D.getPosition(viz.ABS_GLOBAL)
	print experimentObject.currentTrial.obsObj.collisionBox.physNode.body.getPosition()

#vizshape.addAxes().setScale([0.5, 0.5, 0.5])
#vizshape.addBox(size=(0.05,0.05,0.05))
# experimentObject.config.mocap.get_MarkerPos(0,0.5)

