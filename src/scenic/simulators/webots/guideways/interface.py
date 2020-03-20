
import json
import itertools
import math
import re

try:
	import pyproj
except ModuleNotFoundError as e:
	raise RuntimeError('guideways scenarios require pyproj to be installed') from e

import numpy as np

from scenic.core.workspaces import Workspace
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import VectorField
from scenic.core.geometry import polygonUnion, headingOfSegment

def toWebots(point, proj):
	x, z = proj(*point)
	return (-x, z)

def localize(points, proj):
	return tuple(toWebots(pt, proj) for pt in points)

class Bordered:
	def __init__(self, proj, data):
		lefts = data['left_border']
		rights = data['right_border']
		points = lefts + list(reversed(rights))
		self.region = PolygonalRegion(localize(points, proj))

	def show(self, plt, style='r'):
		self.region.show(plt, style=style)

class Guideway(Bordered):
	def __init__(self, proj, data):
		super().__init__(proj, data)
		self.id = data['id']
		self.direction = data['direction']
		self.origin_lane = data['origin_lane']['lane_id']
		medianPoints = localize(data['median'], proj)
		self.median = PolylineRegion(medianPoints)
		self.medianPoints = np.array(medianPoints)
		self.region.orientation = VectorField(f'Road{self.id}Direction', self.directionAt)

	def directionAt(self, point):
		# TODO improve?
		median = self.medianPoints
		pt = np.array(point)
		# find closest point on median
		dists = np.linalg.norm(median - pt, axis=1)
		i, dist = min(enumerate(dists), key=lambda p: p[1])
		# if at either end of median, use that direction
		if i == 0:
			x, y = median[:2]
		elif i == len(median)-1:
			x, y = median[-2:]
		else:
			# otherwise, pick which of the two directions to use based on
			# which side of the angle bisector we are on
			d1, d2 = median[i+1] - median[i], median[i-1] - median[i]
			d1 /= np.linalg.norm(d1)
			d2 /= np.linalg.norm(d2)
			bisector = d1 + d2
			if pt[1] * bisector[0] - pt[0] * bisector[1] < 0:
				x, y = median[i-1], median[i]
			else:
				x, y = median[i], median[i+1]
		return headingOfSegment(x, y)

class Crosswalk(Bordered):
	def show(self, plt, style='b'):
		self.region.show(plt, style=style)

class ConflictZone:
	def __init__(self, inter, data):
		pts = localize(data['polygon']['coordinates'][0], inter.projection)
		self.region = PolygonalRegion(pts)
		# TODO not working... some conflict zones have invalid IDs?
		#self.guideway1 = inter.guidewayWithID[data['guideway1_id']]
		#self.guideway2 = inter.guidewayWithID[data['guideway2_id']]

	def show(self, plt):
		self.region.show(plt, style='k')

def projectionAt(point):
	# taken from Webots OSM importer
	lon, lat = point
	utm_zone = 1 + math.floor((float(lon) + 180) / 6)
	hemisphere = 'south' if lat < 0 else 'north'
	string = "+proj=ortho +%s +zone=%d +lon_0=%f +lat_0=%f +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs" % (hemisphere, utm_zone, lon, lat)
	return pyproj.Proj(string)

class Intersection:
	def __init__(self, data, origin=None):
		if origin is None:
			origin = data['main_gw']['median'][0]
		self.projection = projectionAt(origin)

		self.mainGuideway = Guideway(self.projection, data['main_gw'])
		self.vehicleGuideways = (tuple(Guideway(self.projection, d) for d in data['vehicle_gw'])
								 + (self.mainGuideway,))
		self.bikeGuideways = tuple(Guideway(self.projection, d) for d in data['bicycle_gw'])
		self.railGuideways = tuple(Guideway(self.projection, d) for d in data['rail_gw'])
		guidewayWithID = {}
		for gw in itertools.chain(self.vehicleGuideways, self.bikeGuideways,
								  self.railGuideways):
			guidewayWithID[gw.id] = gw
		self.guidewayWithID = guidewayWithID

		self.crosswalks = tuple(Crosswalk(self.projection, d) for d in data['crosswalks'])
		self.conflictZones = tuple(ConflictZone(self, d) for d in data['conflict_zones'])

	def directionAt(self, point):
		for guideway in self.vehicleGuideways:
			if guideway.region.containsPoint(point):
				return guideway.directionAt(point)
		raise RuntimeError('tried to get road direction at point not in a guideway!')

	def show(self, plt):
		for guideway in self.vehicleGuideways:
			guideway.show(plt)
		self.mainGuideway.show(plt, style='g')
		for guideway in self.bikeGuideways:
			guideway.show(plt, style='c')
		for guideway in self.railGuideways:
			guideway.show(plt, style='m')
		for crosswalk in self.crosswalks:
			crosswalk.show(plt)
		for zone in self.conflictZones:
			zone.show(plt)

class IntersectionWorkspace(Workspace):
	def __init__(self, intersection):
		self.intersection = intersection
		drivablePoly = polygonUnion(gw.region.polygons
									for gw in intersection.vehicleGuideways)
		walkablePoly = polygonUnion(cw.region.polygons
									for cw in intersection.crosswalks)
		workspacePoly = polygonUnion((drivablePoly, walkablePoly))
		self.roadDirection = VectorField('RoadDirection', intersection.directionAt)
		self.drivableRegion = PolygonalRegion(polygon=drivablePoly, orientation=self.roadDirection)
		self.workspaceRegion = PolygonalRegion(polygon=workspacePoly)
		super().__init__(self.workspaceRegion)

	def show(self, plt):
		plt.gca().set_aspect('equal')
		self.intersection.show(plt)

	@property
	def minimumZoomSize(self):
		return 30
