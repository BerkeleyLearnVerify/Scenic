"""Python supporting code for the GTA model."""

import math
import time

import numpy
import scipy.spatial

try:
	import PIL
except ModuleNotFoundError as e:
	raise RuntimeError('GTA scenarios require the PIL module;'
	                   ' try "pip install pillow"') from e

try:
	import cv2
except ModuleNotFoundError as e:
	raise RuntimeError('GTA scenarios require the cv2 module;'
	                   ' try "pip install opencv-python"') from e

import scenic.simulators.gta.center_detection as center_detection
import scenic.simulators.gta.img_modf as img_modf
import scenic.simulators.gta.messages as messages
from scenic.simulators.utils.colors import Color

from scenic.core.distributions import (Samplable, Distribution, Range, Normal, Options,
                                       distributionMethod, toDistribution)
from scenic.core.lazy_eval import valueInContext

from scenic.core.workspaces import Workspace
from scenic.core.vectors import VectorField
from scenic.core.regions import PointSetRegion, GridRegion
import scenic.core.utils as utils
from scenic.core.geometry import *

from scenic.syntax.veneer import verbosePrint

### Abstract GTA interface

class GTA:
	@staticmethod
	def Config(scene):
		ego = scene.egoObject
		vehicles = [GTA.Vehicle(car) for car in scene.objects if car is not ego]
		cameraLoc = GTA.langToGTACoords(ego.position)
		cameraHeading = GTA.langToGTAHeading(ego.heading)

		params = dict(scene.params)
		time = int(round(params.pop('time')))
		minute = time % 60
		hour = int((time - minute) / 60)
		assert hour < 24
		weather = params.pop('weather')
		for param in params:
			print(f'WARNING: unused scene parameter "{param}"')

		return messages.Formal_Config(cameraLoc, [hour, minute], weather,
		                              vehicles, cameraHeading)

	@staticmethod
	def Vehicle(car):
		loc3 = GTA.langToGTACoords(car.position)
		heading = GTA.langToGTAHeading(car.heading)
		scol = list(Color.realToByte(car.color))
		return messages.Vehicle(car.model.name, scol, loc3, heading)
	
	@staticmethod
	def langToGTACoords(point):
		x, y = point
		return [x, y, 60]

	@staticmethod
	def langToGTAHeading(heading):
		h = math.degrees(heading)
		return (h + 360) % 360

### Map

class Map:
	"""Represents roads and obstacles in GTA, extracted from a map image.

	This code handles images from the `GTA V Interactive Map <https://gta-5-map.com/>`_,
	rendered with the "Road" setting.

	Args:
		imagePath (str): path to image file
		Ax (float): width of one pixel in GTA coordinates
		Ay (float): height of one pixel in GTA coordinates
		Bx (float): GTA X-coordinate of bottom-left corner of image
		By (float): GTA Y-coordinate of bottom-left corner of image
	"""
	def __init__(self, imagePath, Ax, Ay, Bx, By):
		self.Ax, self.Ay = Ax, Ay
		self.Bx, self.By = Bx, By
		if imagePath != None:
			startTime = time.time()
			# open image
			image = PIL.Image.open(imagePath)
			self.sizeX, self.sizeY = image.size
			# create version of image for display
			de = img_modf.get_edges(image).convert('RGB')
			self.displayImage = cv2.cvtColor(numpy.array(de), cv2.COLOR_RGB2BGR)
			# detect edges of roads
			ed = center_detection.compute_midpoints(img_data=image, kernelsize=5)
			self.edgeData = {
				self.gridToScenicCoords((x, y)): datum
				for (y, x), datum in ed.items()
			}
			self.orderedCurbPoints = list(self.edgeData.keys())
			# build k-D tree
			self.edgeTree = scipy.spatial.cKDTree(self.orderedCurbPoints)
			# identify points on roads
			self.roadArray = numpy.array(img_modf.convert_black_white(img_data=image).convert('L'),
			                             dtype=int)
			totalTime = time.time() - startTime
			verbosePrint(f'Created GTA map from image in {totalTime:.2f} seconds.')

	@staticmethod
	def fromFile(path):
		startTime = time.time()
		with numpy.load(path, allow_pickle=True) as data:
			Ax, Ay, Bx, By, sizeX, sizeY = data['misc']
			m = Map(None, Ax, Ay, Bx, By)
			m.sizeX, m.sizeY = sizeX, sizeY
			m.displayImage = data['displayImage']
			
			m.edgeData = { tuple(e): center_detection.EdgeData(*rest) for e, *rest in data['edges'] }
			m.orderedCurbPoints = list(m.edgeData.keys())
			m.edgeTree = scipy.spatial.cKDTree(m.orderedCurbPoints)		# rebuild k-D tree

			m.roadArray = data['roadArray']
			totalTime = time.time() - startTime
			verbosePrint(f'Loaded GTA map in {totalTime:.2f} seconds.')
			return m

	def dumpToFile(self, path):
		misc = numpy.array((self.Ax, self.Ay, self.Bx, self.By, self.sizeX, self.sizeY))
		edges = numpy.array([(edge,) + tuple(datum) for edge, datum in self.edgeData.items()])
		roadArray = self.roadArray

		numpy.savez_compressed(path,
			misc=misc, displayImage=self.displayImage,
			edges=edges, roadArray=self.roadArray)

	@property
	@utils.cached
	def roadDirection(self):
		return VectorField('roadDirection', self.roadHeadingAt)
	
	@property
	@utils.cached
	def roadRegion(self):
		return GridRegion('road', self.roadArray,
		                  self.Ax, self.Ay, self.Bx, self.By,
		                  orientation=self.roadDirection)

	@property
	@utils.cached
	def curbRegion(self):
		return PointSetRegion('curb', self.orderedCurbPoints,
		                      kdTree=self.edgeTree,
		                      orientation=self.roadDirection)

	def gridToScenicCoords(self, point):
		x, y = point[0], point[1]
		return ((self.Ax * x) + self.Bx, (self.Ay * y) + self.By)

	def gridToScenicHeading(self, heading):
		return heading - (math.pi / 2)

	def scenicToGridCoords(self, point):
		x, y = point[0], point[1]
		return ((x - self.Bx) / self.Ax, (y - self.By) / self.Ay)

	def scenicToGridHeading(self, heading):
		return heading + (math.pi / 2)

	@distributionMethod
	def roadHeadingAt(self, point):
		# find closest edge
		distance, location = self.edgeTree.query(point)
		closest = tuple(self.edgeTree.data[location])
		# get direction of edge
		return self.gridToScenicHeading(self.edgeData[closest].tangent)

	def show(self, plt):
		plt.imshow(self.displayImage)

class MapWorkspace(Workspace):
	"""Workspace whose rendering is handled by a Map"""
	def __init__(self, mappy, region):
		super().__init__(region)
		self.map = mappy

	def scenicToSchematicCoords(self, coords):
		return self.map.scenicToGridCoords(coords)

	def show(self, plt):
		return self.map.show(plt)

	@property
	def minimumZoomSize(self):
		return 40 / min(abs(self.map.Ax), abs(self.map.Ay))

### Car models and colors

class CarModel:
	"""A model of car in GTA.

	Attributes:
		name (str): name of model in GTA
		width (float): width of this model of car
		length (float): length of this model of car
		viewAngle (float): view angle in radians (default is 90 degrees)

	Class Attributes:
		models: dict mapping model names to the corresponding `CarModel`
	"""

	def __init__(self, name, width, length, viewAngle=math.radians(90)):
		super(CarModel, self).__init__()
		self.name = name
		self.width = width
		self.length = length
		self.viewAngle = viewAngle

	@classmethod
	def uniformModel(self):
		return Options(self.modelProbs.keys())

	@classmethod
	def egoModel(self):
		return self.models['BLISTA']

	@classmethod
	def defaultModel(self):
		return Options(self.modelProbs)

	def __str__(self):
		return f'<CarModel {self.name}>'

CarModel.modelProbs = {
	CarModel('BLISTA', 1.75871, 4.10139): 1,
	CarModel('BUS', 2.9007, 13.202): 0,
	CarModel('NINEF', 2.07699, 4.50658): 1,
	CarModel('ASEA', 1.83066, 4.45861): 1,
	CarModel('BALLER', 2.10791, 5.10333): 1,
	CarModel('BISON', 2.29372, 5.4827): 1,
	CarModel('BUFFALO', 2.04265, 5.07782): 1,
	CarModel('BOBCATXL', 2.37944, 5.78222): 1,
	CarModel('DOMINATOR', 1.9353, 4.9355): 1,
	CarModel('GRANGER', 3.02698, 5.94577): 1,
	CarModel('JACKAL', 2.00041, 4.91436): 1,
	CarModel('ORACLE', 2.07787, 5.12544): 1,
	CarModel('PATRIOT', 2.26679, 5.13695): 1,
	CarModel('PRANGER', 3.02698, 5.94577): 1
}
CarModel.models = { model.name: model for model in CarModel.modelProbs }

CarColor = Color 	# for backwards compatibility
