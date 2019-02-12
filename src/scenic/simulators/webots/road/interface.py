
import time
import sys
import math
import itertools

import numpy as np
import shapely.geometry
import shapely.ops

import scenic.simulators.webots.world_parser as world_parser
from scenic.core.workspaces import Workspace
from scenic.core.vectors import PolygonalVectorField
from scenic.core.regions import PolygonalRegion, PolylineRegion, nowhere
from scenic.core.geometry import (normalizeAngle, rotateVector, polygonUnion, cleanChain,
                                  plotPolygon)
from scenic.syntax.veneer import verbosePrint

def polygonWithPoints(points):
	polygon = shapely.geometry.Polygon(points)
	if not polygon.is_valid:	# TODO improve hack?
		verbosePrint(f'WARNING: simplifying invalid polygon with points {points}')
		polygon = polygon.simplify(0.5)
		if not polygon.is_valid:
			raise RuntimeError(f'unable to simplify polygon {polygon}')
	return polygon

def regionWithPolygons(polygons, orientation=None):
	if len(polygons) == 0:
		return nowhere
	else:
		return PolygonalRegion(polygon=polygons, orientation=orientation)

## Classes for WBT nodes we are interested in

class OSMObject:
	"""Objects with OSM id tags"""
	def __init__(self, attrs):
		self.attrs = attrs
		self.osmID = attrs['id']

class Road(OSMObject):
	"""OSM roads"""
	def __init__(self, attrs, driveOnLeft=False):
		super().__init__(attrs)
		self.driveOnLeft = driveOnLeft
		self.translation = attrs['translation']
		pts = [np.delete(p + self.translation, 1) for p in attrs['wayPoints']]
		self.waypoints = tuple(cleanChain(pts, 0.05))
		assert len(self.waypoints) > 1, pts
		self.width = float(attrs.get('width', 7))
		self.lanes = int(attrs.get('numberOfLanes', 2))
		if self.lanes < 1:
			raise RuntimeError(f'Road {self.osmID} has fewer than 1 lane!')
		self.forwardLanes = int(attrs.get('numberOfForwardLanes', 1))
		# if self.forwardLanes < 1:
		# 	raise RuntimeError(f'Road {self.osmID} has fewer than 1 forward lane!')
		self.backwardLanes = self.lanes - self.forwardLanes
		self.hasLeftSidewalk = attrs.get('leftBorder', True)
		self.hasRightSidewalk = attrs.get('rightBorder', True)
		self.sidewalkWidths = list(attrs.get('roadBorderWidth', [0.8]))
		if ((self.hasLeftSidewalk or self.hasRightSidewalk)
				and len(self.sidewalkWidths) < 1):
			raise RuntimeError(f'Road {self.osmID} has sidewalk with empty width!')
		self.startCrossroad = attrs.get('startJunction')
		self.endCrossroad = attrs.get('endJunction')

	def computeGeometry(self, crossroads, snapTolerance=0.05):
		## Approximate bounding polygon and sidewalks
		# TODO improve this!!!
		lefts, rights = [], []
		leftSidewalk, rightSidewalk = [], []
		headings = []
		sidewalkWidths = itertools.chain(self.sidewalkWidths,
		                                 itertools.repeat(self.sidewalkWidths[-1]))
		segments = zip(self.waypoints, sidewalkWidths)
		for i, segment in enumerate(segments):
			point, sidewalkWidth = segment
			if i+1 < len(self.waypoints):
				nextPt = self.waypoints[i+1]
				dx, dy = nextPt - point
				heading = normalizeAngle(math.atan2(dy, dx) - (math.pi / 2))
				headings.append(heading)
				perp = np.array([-dy, dx])
				perp /= np.linalg.norm(perp)
			else:
				pass	# use perp from last segment
			toEdge = perp * (self.width / 2)
			left = point + toEdge
			right = point - toEdge
			lefts.append(left)
			rights.append(right)
			toEdge = perp * sidewalkWidth
			leftSidewalk.append(left + toEdge)
			rightSidewalk.append(right - toEdge)

		# Snap to adjacent crossroads if possible
		if snapTolerance > 0:
			sc = self.startCrossroad
			if sc is not None:
				if sc not in crossroads:
					raise RuntimeError(f'Road {self.osmID} begins at invalid crossroad {sc}')
				crossroad = crossroads[sc]
				if crossroad.region is not None:
					pt = shapely.geometry.Point(lefts[0])
					pt = shapely.ops.snap(pt, crossroad.region.polygons, snapTolerance)
					lefts[0] = np.array([pt.x, pt.y])
					pt = shapely.geometry.Point(rights[0])
					pt = shapely.ops.snap(pt, crossroad.region.polygons, snapTolerance)
					rights[0] = np.array([pt.x, pt.y])
					perp = lefts[0] - rights[0]
					toEdge = perp * (self.sidewalkWidths[0] / np.linalg.norm(perp))
					leftSidewalk[0] = lefts[0] + toEdge
					rightSidewalk[0] = rights[0] - toEdge
			ec = self.endCrossroad
			if ec is not None:
				if ec not in crossroads:
					raise RuntimeError(f'Road {self.osmID} ends at invalid crossroad {ec}')
				crossroad = crossroads[ec]
				if crossroad.region is not None:
					pt = shapely.geometry.Point(lefts[-1])
					pt = shapely.ops.snap(pt, crossroad.region.polygons, snapTolerance)
					lefts[-1] = np.array([pt.x, pt.y])
					pt = shapely.geometry.Point(rights[-1])
					pt = shapely.ops.snap(pt, crossroad.region.polygons, snapTolerance)
					rights[-1] = np.array([pt.x, pt.y])
					perp = lefts[-1] - rights[-1]
					toEdge = perp * (self.sidewalkWidths[-1] / np.linalg.norm(perp))
					leftSidewalk[-1] = lefts[-1] + toEdge
					rightSidewalk[-1] = rights[-1] - toEdge

		roadPoints = lefts + list(reversed(rights))
		self.leftCurb = PolylineRegion(reversed(lefts))
		self.rightCurb = PolylineRegion(rights)

		self.leftSidewalk = self.rightSidewalk = None
		if self.hasLeftSidewalk:
			points = lefts + list(reversed(leftSidewalk))
			polygon = polygonWithPoints(points)
			assert polygon.is_valid, self.waypoints
			self.leftSidewalk = PolygonalRegion(polygon=polygon)
		if self.hasRightSidewalk:
			points = rights + list(reversed(rightSidewalk))
			polygon = polygonWithPoints(points)
			assert polygon.is_valid, self.waypoints
			self.rightSidewalk = PolygonalRegion(polygon=polygon)

		## Compute lanes and traffic directions
		cells = []
		la, ra = lefts[0], rights[0]
		gapA = (ra - la) / self.lanes
		markerA = ra
		laneMarkers = [[] for lane in range(self.lanes)]
		for lb, rb, heading in zip(lefts[1:], rights[1:], headings):
			# Compute lanes for this segment of road
			gapB = (rb - lb) / self.lanes
			markerB = rb
			for lane, markers in enumerate(laneMarkers):
				forward = lane < self.forwardLanes
				if self.driveOnLeft:
					forward = not forward
				nextMarkerA = markerA - gapA
				nextMarkerB = markerB - gapB
				markers.append(nextMarkerA)
				cell = shapely.geometry.Polygon((markerA, markerB, nextMarkerB, nextMarkerA))
				heading = heading if forward else normalizeAngle(heading + math.pi)
				cells.append((cell, heading))
				markerA = nextMarkerA
				markerB = nextMarkerB
			gapA = gapB
			markerA = rb
		self.lanes = []
		markerB = rb
		rightEdge = rights
		for lane, markers in enumerate(laneMarkers):
			markerB = markerB - gapB
			markers.append(markerB)
			self.lanes.append(PolygonalRegion(rightEdge + list(reversed(markers))))
			rightEdge = markers
		self.laneMarkers = laneMarkers[:-1]
		self.cells = cells
		self.direction = PolygonalVectorField(f'Road{self.osmID}Direction', cells)

		roadPolygon = polygonWithPoints(roadPoints)
		self.region = PolygonalRegion(polygon=roadPolygon, orientation=self.direction)

	def show(self, plt):
		if self.hasLeftSidewalk:
			x, y = zip(*self.leftSidewalk.points)
			plt.fill(x, y, '#A0A0FF')
		if self.hasRightSidewalk:
			x, y = zip(*self.rightSidewalk.points)
			plt.fill(x, y, '#A0A0FF')
		self.region.show(plt, style='r:')
		x, y = zip(*self.lanes[0].points)
		plt.fill(x, y, color=(0.8, 1.0, 0.8))
		for lane, markers in enumerate(self.laneMarkers):
			x, y = zip(*markers)
			color = (0.8, 0.8, 0) if lane == self.backwardLanes - 1 else (0.3, 0.3, 0.3)
			plt.plot(x, y, '--', color=color)

class Crossroad(OSMObject):
	"""OSM crossroads"""
	def __init__(self, attrs):
		super().__init__(attrs)
		self.translation = attrs['translation']
		points = list(p + self.translation for p in attrs['shape'])
		if len(points) > 0:
			self.points = [(x, z) for x, y, z in points]
			self.region = PolygonalRegion(self.points)
		else:
			verbosePrint(f'WARNING: Crossroad {self.osmID} has empty shape field!')
			self.region = None

	def show(self, plt):
		if self.region is not None:
			x, y = zip(*self.points)
			plt.fill(x, y, color=(1, 0.9, 0.9))
			plt.plot(x, y, ':', color=(1, 0.5, 0))

class PedestrianCrossing:
	"""PedestrianCrossing nodes"""
	def __init__(self, attrs):
		self.translation = attrs.get('translation', np.array((0, 0, 0)))
		pos = np.delete(self.translation, 1)
		name = attrs.get('name', '')
		rotation = attrs.get('rotation', (0, 1, 0, 0))
		if tuple(rotation[:3]) != (0, 1, 0):
			raise RuntimeError(f'PedestrianCrossing "{name}" is not 2D!')
		self.angle = float(rotation[3])
		size = attrs.get('size', (20, 8))
		self.length, self.width = float(size[0]), float(size[1])
		self.length += 0.2		# pad length to intersect sidewalks better	# TODO improve?
		hl, hw = self.length / 2, self.width / 2
		self.corners = tuple(pos + rotateVector(vec, -self.angle)
			for vec in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)))
		self.region = PolygonalRegion(self.corners)

	def show(self, plt):
		x, y = zip(*self.corners)
		plt.fill(x, y, color='#A0A0FF')

## Workspace derived from a WBT world

class WebotsWorkspace(Workspace):
	def __init__(self, world):
		# Find roads, crossroads, and pedestrian crossings
		nodeClasses = {
			'Road': Road,
			'Crossroad': Crossroad,
			'PedestrianCrossing': PedestrianCrossing
		}
		self.roads, self.crossroads, self.crossings = world_parser.findNodeTypesIn(
			('Road', 'Crossroad', 'PedestrianCrossing'), world, nodeClasses)

		# Compute road geometry
		crossroadsByID = { crossroad.osmID: crossroad for crossroad in self.crossroads }
		for road in self.roads:
			road.computeGeometry(crossroadsByID)

		# Construct regions
		allCells = []
		for road in self.roads:
			assert road.region.polygons.is_valid, (road.waypoints, road.region.points)
			allCells.extend(road.cells)
		for crossroad in self.crossroads:
			if crossroad.region is not None:
				for poly in crossroad.region.polygons:
					allCells.append((poly, None))
		self.roadDirection = PolygonalVectorField(
		    'roadDirection', allCells,
		    headingFunction=lambda pos: 0, defaultHeading=0		# TODO fix
		)
		roadPoly = polygonUnion(road.region.polygons for road in self.roads)
		self.roadsRegion = PolygonalRegion(polygon=roadPoly,
		                                   orientation=self.roadDirection)
		crossroadPoly = polygonUnion(cr.region.polygons for cr in self.crossroads
		                             if cr.region is not None)
		self.crossroadsRegion = PolygonalRegion(polygon=crossroadPoly,
		                                        orientation=self.roadDirection)

		sidewalks = []
		for road in self.roads:
			if road.hasLeftSidewalk:
				sidewalks.append(road.leftSidewalk.polygons)
			if road.hasRightSidewalk:
				sidewalks.append(road.rightSidewalk.polygons)
		sidewalksPoly = polygonUnion(sidewalks)
		self.sidewalksRegion = regionWithPolygons(sidewalksPoly)

		crossingsPoly = polygonUnion(crossing.region.polygons
		                             for crossing in self.crossings)
		self.crossingsRegion = regionWithPolygons(crossingsPoly)

		walkablePoly = polygonUnion((sidewalksPoly, crossingsPoly))
		self.walkableRegion = regionWithPolygons(walkablePoly)
		drivablePoly = polygonUnion((roadPoly, crossroadPoly))
		self.drivableRegion = PolygonalRegion(polygon=drivablePoly,
		                                      orientation=self.roadDirection)
		workspacePoly = polygonUnion((drivablePoly, walkablePoly))
		self.workspaceRegion = PolygonalRegion(polygon=workspacePoly)

		# Identify various roads and lanes of interest
		slowCurbs = []
		for road in self.roads:
			if road.forwardLanes > 1:
				slowCurbs.append(road.rightCurb)
			if road.backwardLanes > 1:
				slowCurbs.append(road.leftCurb)
		self.slowCurbs = slowCurbs

		super().__init__(self.workspaceRegion)

	def show(self, plt):
		plt.gca().set_aspect('equal')
		self.drivableRegion.show(plt)
		for road in self.roads:
			road.show(plt)
		for crossroad in self.crossroads:
			crossroad.show(plt)
		for crossing in self.crossings:
			crossing.show(plt)
		for curb in self.slowCurbs:
			curb.show(plt, style='b-')

	@property
	def minimumZoomSize(self):
		return 30
