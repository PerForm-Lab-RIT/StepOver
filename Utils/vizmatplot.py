import viz
import threading
import vizact
import time
import viztask

##IMPORTANT: Need this so Python threads are given time to run
viz.directormode(viz.DIRECTOR_FAST)

class Show:
	def __init__(self, fig):
		self.canvasData = viz.Data(lock = threading.Lock(), havenewData=False )	
		self.fig = fig
		self.t = threading.Thread(target=self.computeFigureThread)
		self.t.start()

		#IMPORTANT: Wait for thread to finish before exiting
		vizact.onexit(self.t.join)
		#Create a blank texture to display canvas data
		self.tex = viz.addBlankTexture([1,1])
		#Create onscreen quad to display texture
		self.quad = viz.addTexQuad(parent=viz.ORTHO,texture=self.tex)
		self.quad.alpha(0.5)
		self.link = viz.link(viz.MainWindow.CenterCenter,self.quad)
	#	self.link.setOffset([400,200,0])
		self.drawer = vizact.ontimer(0, self.drawPlot)
		self.rate_ctr = time.time()
		self.drawrate_txt = viz.addText('', viz.SCREEN)
		self.drawrate_txt.setPosition(0,0.01)
		self.drawrate_txt.scale(.7,.7)
		self.drawrate_txt.color(viz.WHITE)
		self.drawrate_txt.visible(0)
		self.rate = 0
		#scale variables to change size of plot image
		self.scale_x = 1
		self.scale_y = 1
		
	def computeFigureThread(self):
		while not viz.done():
		#	viz.waitframe(2)
			self.fig.canvas.draw() #draw the plot
			self.canvasData.lock.acquire()
			self.canvasData.imageData = self.fig.canvas.tostring_rgb() #store image data to apply to texture
			self.canvasData.imageSize = self.fig.canvas.get_width_height()
			self.canvasData.havenewData = True
			self.canvasData.lock.release()
	
	def drawPlot(self):	
	#Update texture with latest canvas data
		if self.canvasData.havenewData:
			self.canvasData.lock.acquire()
			self.tex.setImageData( self.canvasData.imageData, self.canvasData.imageSize )
			self.quad.setScale(self.canvasData.imageSize[0]*self.scale_x,-self.canvasData.imageSize[1]*self.scale_y,1) # Negate Y scale since canvas image data uses upper-left origin
			self.canvasData.havenewData = False
			self.canvasData.lock.release()

			self.rate = time.time()-self.rate_ctr
			try:
				self.drawrate_txt.message('Draw Rate: ' + '%5.2f'%(1/self.rate) + 'hz' )
			except ZeroDivisionError:
				print 'boo!'
			#print self.rate
			self.rate_ctr = time.time()
			
	def showDrawRate(self, arg=True):
		if arg == True:
			self.drawrate_txt.visible(1)
		else:
			self.drawrate_txt.visible(0)
	
	def setScale(self, x=1, y=1):
		#changes scale of plot image
		self.scale_x = x
		self.scale_y = y
	
	def setOffset(self, x=0, y=0):
		#changes offset of quad link
		self.link.setOffset([x,y,0])
			