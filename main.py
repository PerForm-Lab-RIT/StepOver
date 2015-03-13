"""
Runs an experiment.
"""

import viz
import viztask
import vrlabConfig
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

class Experiment(viz.EventClass):
	
	"""
	Experiment manages the basic operation of the experiment.
	"""
	
	def __init__(self, config):
		
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
		
		self.config = config 
		
		# Update to reflect actual leg length of user
		self.inputLegLength()

		# Eventually, self.config.writables is passed to DVRwriter
		# self.config.writables is a list
		# dvrwriter will attempt to run .getOutput on every member of the list
		# One could then add experiment, theBall, theRacquet, eyeTrackingCal to the list, assuming 
		# they include the member function .getOutput().
		# I prefer to do all my data collection in one place: experiment.getOutput()
		
		self.config.writables = [self]
		
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
		
		self.collisionLocOnObs_XYZ = [nan,nan,nan]
		
		################################################################
		##  LInk up the hmd to the mainview
		
		if( self.config.use_phasespace == True and self.config.use_HMD ):
			
			################################################################
			##  Link up the hmd to the mainview
			if( config.mocap.returnPointerToRigid('hmd') ):
				self.config.mocap.enableHMDTracking()
	
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
					print 'LBP :> Remove Obs'
					
					#self.currentTrial.metronomeTimerObj.setEnabled(viz.TOGGLE);
					#self.currentTrial.metronomeTimerObj = [];
				
					self.currentTrial.goSignalTimerObj.setEnabled(viz.TOGGLE);
					self.currentTrial.goSignalTimerObj = [];
				
				##############################################
				### Go signal already given.  Starting the trial
				elif(self.currentTrial.goSignalGiven is True):
					
					print 'Starting trial ==> Type', self.currentTrial.trialType
					self.currentTrial.approachingObs = True
					
					# Start data collection
					viz.playSound(soundBank.bubblePop)
					
					if( type(self.currentTrial.goSignalTimerObj) is not list ):			
						self.currentTrial.goSignalTimerObj.remove()
	
					vizact.ontimer2(self.maxTrialDuration, 0,self.endTrial)
					
										
					#self.maxTrialDurationObj
					#self.trialTimeoutTimerID
				
	def _checkForCollisions(self):
		
		thePhysEnv = self.room.physEnv;
		
		if( thePhysEnv.collisionDetected == False ): 
			# No collisions this time!
			return
		
		leftFoot = self.room.leftFoot
		rightFoot = self.room.rightFoot
		
		if( self.currentTrial.approachingObs == True ):
			
			obstacle = self.currentTrial.obsObj
			
			for idx in range(len(thePhysEnv.collisionList_idx_physNodes)):
				
				physNode1 = thePhysEnv.collisionList_idx_physNodes[idx][0]
				physNode2 = thePhysEnv.collisionList_idx_physNodes[idx][1]
				
				if( physNode1 == leftFoot.physNode and physNode2 == obstacle.physNode):

					self.eventFlag.setStatus(4)
					if ( obstacle.physNode.collisionPosLocal_XYZ):
						self.collisionLocOnObs_XYZ = obstacle.physNode.collisionPosLocal_XYZ
					elif( leftFoot.physNode.collisionPosLocal_XYZ ):
						self.collisionLocOnObs_XYZ = leftFoot.physNode.collisionPosLocal_XYZ
					print 'Collided Objects are Left Foot and Obstacle at\n', self.collisionLocOnObs_XYZ
					#self.currentTrial.removeObs()
					viz.playSound(soundBank.beep)
				
				elif( physNode1 == rightFoot.physNode and physNode2 == obstacle.physNode ):
					
					self.eventFlag.setStatus(5)
					if ( obstacle.physNode.collisionPosLocal_XYZ ):
						self.collisionLocOnObs_XYZ = obstacle.physNode.collisionPosLocal_XYZ
					elif (rightFoot.physNode.collisionPosLocal_XYZ):
						self.collisionLocOnObs_XYZ = rightFoot.physNode.collisionPosLocal_XYZ
					print 'Collided Objects are Right Foot and Obstacle at\n', self.collisionLocOnObs_XYZ
					#self.currentTrial.removeObs()
					viz.playSound(soundBank.beep)
			
	def start(self):
		
		##This is called when the experiment should begin.
		self.setEnabled(True)
		self.config.start()

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
		if key == 'm':
			print 'Data Recording Starts \n'
			self.expInProgress = True;
			
		
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
		#
		outputString = outputString + ' eventFlag %f ' % (self.eventFlag.status)
		#if ( ( self.eventFlag == 3 ) or ( self.eventFlag == 4 ) or ( self.eventFlag == 5 ) ):
		
		#FIXME: Collision locatoin is not set correctly in _checkForCollisions
		#if( self.eventFlag.status == 4 or self.eventFlag.status == 5 ):
		outputString = outputString + ' collisionLocOnObs_XYZ [ %f %f %f ] ' % (self.collisionLocOnObs_XYZ[0], self.collisionLocOnObs_XYZ[1], self.collisionLocOnObs_XYZ[2])
		
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
		
		## =======================================================================================================
		## Left and Right Foot Position
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
		#outputString = outputString + '< RightFootQUAT_WXYZ %f %f %f %f > ' % ( rightFootQUAT_XYZW[0], rightFootQUAT_XYZW[1], rightFootQUAT_XYZW[2], rightFootQUAT_XYZW[3] )
		
		
		MarkerPos = []
		
		#FIXME: Hardcoded foot markers numbers for data output.
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
		
		if( self.room.rightFoot ):
			
			leftFootPos_XYZ = self.room.leftFoot.visNode.getPosition()
			leftFootMat = self.room.leftFoot.visNode.getMatrix()
			leftFootQUAT_XYZW = leftFootMat.getQuat()
			
		else:
			leftFootPos_XYZ = [None, None, None]
			leftFootQUAT_XYZW = [None, None, None]
			
		#outputString = outputString + '[ LeftFoot_XYZ %f %f %f ] ' % (leftFootPos_XYZ[0], leftFootPos_XYZ[1], leftFootPos_XYZ[2])
		
		#outputString = outputString + '< LeftFootQUAT_WXYZ %f %f %f %f > ' % ( leftFootQUAT_XYZW[0], leftFootQUAT_XYZW[1], leftFootQUAT_XYZW[2], leftFootQUAT_XYZW[3] )
		
		return outputString #%f %d' % (viz.getFrameTime(), self.inCalibrateMode)
		
	def getEyeData(self):
		
		# Legend:
		# ** for 1 var
		# () for 2 vars
		# [] for 3 vars
		# <> for 4 vars
		# @@ for 16 vars (view and projection matrices)

		outputString = ''		

		class VPX_RealType(Structure):
			 _fields_ = [("x", c_float),("y", c_float)]

		## Position
		arrington = self.config.eyeTrackingCal.arrington;
		
		#eyePos_XY = pyArr.getPosition(self.arrington.RAW)
		#outputString = outputString + '( eyePos_XY %f %f ) ' %(eyePos_XY[0],eyePos_XY[1])
		
		Eye_A  = c_int(0)

		eyePos = VPX_RealType()
		eyePosPointer = pointer(eyePos)
		
		arrington.VPX_GetGazePoint2(Eye_A ,eyePosPointer)
		outputString = outputString + '( eyePos %f  %f ) ' % (eyePos.x, eyePos.y)
		
		## Time
		eyeTime = c_double();
		eyeTimePointer = pointer(eyeTime)		
		arrington.VPX_GetDataTime2(Eye_A ,eyeTimePointer)
		outputString = outputString + '* eyeDataTime %f * ' % eyeTime.value
		
		## Quality
		eyeQual = c_int();
		eyeQualPointer = pointer(eyeQual)		
		arrington.VPX_GetDataQuality2(Eye_A ,eyeQualPointer)
		outputString = outputString + '* eyeQuality %i * ' % eyeQual.value
		
		return outputString

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
			self.room.standingBox.setPosition([0.0, 0.1, -3.5])
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
			print 'End of Trial :> Attempting to remove Obs'
			self.currentTrial.removeObs()
			print 'End of Trial :> Remove Obs'
		
			## Stop timers
			if( type(self.currentTrial.metronomeTimerObj) is not list ):			
				self.currentTrial.metronomeTimerObj.remove()
			
			if( type(self.currentTrial.goSignalTimerObj) is not list ):			
				self.currentTrial.goSignalTimerObj.remove()
			
			if( type(self.currentTrial.trialTimeoutTimerObj) is not list ):			
				self.currentTrial.trialTimeoutTimerObj.remove()
			
			self.eventFlag.setStatus(6)
			
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
		if( (self.currentTrial.approachingObs == False) ): # self.expInProgress is False) or (
			return
	
		expDataString = self.getOutput()
		self.expDataFile.write(expDataString+ '\n')
					
		# Eyetracker data
	
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
		
		#self.currentTrial.metronomeTimerObj = vizact.ontimer(self.metronomeTimeMS/1000,self.metronomeHighTic)
		
	def inputLegLength(self):
		#print('SETTING LEG LENGTH TO 100!')
		#self.config.obsHeightLegRatio = 100

		#prompt for a string
		self.config.obsHeightLegRatio = vizinput.input('Enter leg length (cm):', value = float(90))
		
		try:
			# Test if it's an integer
			intValue = int(self.config.obsHeightLegRatio)
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
		leftFoot.setMocapRigidBody(config.mocap,'leftFoot')
		leftFoot.toggleUpdateWithRigid()
		leftFoot.enablePhysNode()
		leftFoot.toggleUpdatePhysWithVis()
		#leftFoot.visNode.disable(viz.RENDERING)
		leftFoot.visNode.visible(viz.ON)

		
		rightFoot = self.room.rightFoot
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

		
		markerZVals_mIdx = []
		markerXVals_mIdx= []
		
		#footHeightFromGround = 0 
		footRigid = mocap.returnPointerToRigid(footSide)
		markerID_midx = footRigid.markerID_midx
		
		for mIdx in range(len(markerID_midx)):
			
			vizXYZ = mocap.getMarkerPosition(markerID_midx[mIdx])
			
			if( vizXYZ == 0):
				print 'Could not see all markers on rigid body'
				return
				
			markerXVals_mIdx.append(vizXYZ[0])
			markerZVals_mIdx.append(vizXYZ[2])
	
		
		sumOfMarkerHeights = 0
		
		for mIdx in footRigid.avgMarkerList_midx:
			vizXYZ = mocap.getMarkerPosition(markerID_midx[mIdx])
			sumOfMarkerHeights = sumOfMarkerHeights + vizXYZ[1]
			
		avgMarkerHeight = sumOfMarkerHeights / len(footRigid.avgMarkerList_midx)
		
		# one centimeter  buffer
		footLength = max(markerXVals_mIdx) - min(markerXVals_mIdx) - .02
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
		
		# 1 
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
			
			#print 'Setting status to' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
			
		elif( overWriteBool is True and self.lastFrameUpdated == viz.getFrameNumber() ):
			
			#print 'Overwrite from status ' + str(self.status) + ' to ' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
		
			
		elif( self.lastFrameUpdated == viz.getFrameNumber() ):
			
		#print 'Stopped attempt to overwrite status of ' + str(self.status) + ' with ' + str(status) + ' [overWriteBool=False]'
			pass
		
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
		
		self.legLengthCM = config.expCfg['experiment']['legLengthCM']		
		self.obsHeightM = []
		
		# Object placeholders
		self.obsObj = -1
		self.objectSizeText = -1
		
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
		
		#print 'Creating object at ' + str(obsLoc)
		
		self.obsHeightM = self.legLengthCM * self.obsHeightLegRatio / 100
		obsSize = [0.1,1,self.obsHeightM] # lwh
		obsLoc = [self.obsXLoc,self.obsHeightM/2,self.obsZLoc]
		
		self.obsObj = visEnv.visObj(room,'box',obsSize,obsLoc,self.obsColor_RGB)
		self.obsObj.enablePhysNode()
		
		
		if( self.objIsVirtual == False ):
			
			self.obsObj.visNode.disable(viz.RENDERING)
			# Draw a line on the ground
			lineHeight = 0.001
			lineSize = [0.01,1,lineHeight] # lwh
			obsLoc = [self.obsXLoc,lineHeight/2,self.obsZLoc]
			self.lineObj = visEnv.visObj(room,'box',lineSize,obsLoc,self.obsColor_RGB)
			
			if( self.trialType == '	t4' ):
				displayText = 'Short'
				#print 'Obs Height =>', displayText, self.trialType
			elif( self.trialType == 't5' ):
				displayText = 'Med'
				#print 'Obs Height =>', displayText, self.trialType
			elif( self.trialType == 't6' ):
				displayText = 'Tall'
				#print 'Obs Height =>', displayText, self.trialType
			else:
				print 'Object height invalid!'
				return
				
			self.objectSizeText = viz.addText3D(displayText)
			self.objectSizeText.setEuler([-90,90,0], viz.ABS_GLOBAL)
			self.objectSizeText.setPosition([-1.,.001,1.3],viz.ABS_GLOBAL)
			scale = 0.1
			self.objectSizeText.setScale([scale ,scale ,scale])
		else:
			print'Placing Obstacle at', obsLoc, 'size', obsSize
		###############
#		if( self.objIsVirtual == True ):
#			
#			self.obsHeightM = self.legLengthCM * self.obsHeightLegRatio / 100
#			obsSize = [0.1,1,self.obsHeightM] # lwh
#			obsLoc = [self.obsXLoc,self.obsHeightM/2,self.obsZLoc]
#					
#			#self.obsObj = visEnv.visObj(room,'box',obsSize,obsLoc,self.obsColor_RGB)
#		else:
#			
#			if( self.trialType == '	t4' ):
#				displayText = 'Short'
#				print 'Obs Height =>', displayText, self.trialType
#			elif( self.trialType == 't5' ):
#				displayText = 'Med'
#				print 'Obs Height =>', displayText, self.trialType
#			else: # ( self.trialType == 't6' )
#				displayText = 'Tall'
#				print 'Obs Height =>', displayText, self.trialType
#			
#			# Fix Me : Kamran : It seems here the Height is being overwritten which is wrong
#			#self.obsHeightM = 0.001
#			self.obsHeightM = self.legLengthCM * self.obsHeightLegRatio / 100
#			obsSize = [0.1,1,self.obsHeightM] # lwh
#			obsLoc = [self.obsXLoc,self.obsHeightM/2,self.obsZLoc]
#
#			self.objectSizeText = viz.addText3D(displayText)
#			self.objectSizeText.setEuler([-90,90,0], viz.ABS_GLOBAL)
#			self.objectSizeText.setPosition([-1.,.001,1.3],viz.ABS_GLOBAL)
#			scale = 0.1
#			self.objectSizeText.setScale([scale ,scale ,scale])
#
#		if ( self.room.isWalkingUpAxis ):
#			obsLoc[2] = obsLoc[2] - self.room.offsetDistance + self.room.standingBox.getPosition()[2]
#		else:
#			obsLoc[2] = obsLoc[2] + self.room.offsetDistance + self.room.standingBox.getPosition()[2]
#
#		
#		self.obsObj = visEnv.visObj(room,'box',obsSize,obsLoc,self.obsColor_RGB)
#		self.obsObj.enablePhysNode()
#		

	def removeObs(self):

		if( self.objectSizeText != -1 and (self.trialType == 't4' or self.trialType == 't5'  or self.trialType == 't6' )):
			self.objectSizeText.remove()
			self.objectSizeText = -1
			
		if( self.obsObj != -1):
			self.obsObj.remove()		
			self.obsObj = -1
			print 'removeObs: Obstacle removed'
		
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

experimentConfiguration = vrlabConfig.VRLabConfig(expConfigFileName)


## vrlabConfig uses config to setup hardware, motion tracking, frustum, eyeTrackingCal.
##  This draws upon the system config to setup the hardware / HMD

## The experiment class initialization draws the room, sets up physics, 
## and populates itself with a list of blocks.  Each block contains a list of trials

experimentObject = Experiment(experimentConfiguration)
experimentObject.start()
#
#demoMode(experimentObject)
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
	
	

	
	
	