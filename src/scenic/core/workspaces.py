"""Workspaces."""

from scenic.core.distributions import needsSampling
from scenic.core.regions import Region, everywhere
from scenic.core.geometry import findMinMax
from scenic.core.utils import RuntimeParseError

class Workspace(Region):
	"""A workspace describing the fixed world of a scenario"""
	def __init__(self, region=everywhere):
		if needsSampling(region):
			raise RuntimeParseError('workspace region must be fixed')
		super().__init__('workspace', orientation=region.orientation)
		self.region = region

	def langToMapCoords(self, coords):
		"""Converts language to workspace coordinates"""
		return coords

	def show(self, plt):
		"""Render a schematic of the workspace for debugging"""
		try:
			aabb = self.region.getAABB()
		except NotImplementedError:		# unbounded Regions don't support this
			return
		((xmin, ymin), (xmax, ymax)) = aabb
		plt.xlim(xmin, xmax)
		plt.ylim(ymin, ymax)
		plt.gca().set_aspect('equal')

	def zoomAround(self, plt, objects, expansion=2):
		"""Zoom the schematic around the specified objects"""
		positions = (self.langToMapCoords(obj.position) for obj in objects)
		x, y = zip(*positions)
		minx, maxx = findMinMax(x)
		miny, maxy = findMinMax(y)
		sx = expansion * (maxx - minx)
		sy = expansion * (maxy - miny)
		s = max(sx, sy, self.minimumZoomSize) / 2.0
		s += max(max(obj.width, obj.height) for obj in objects)	# TODO improve
		cx = (maxx + minx) / 2.0
		cy = (maxy + miny) / 2.0
		plt.xlim(cx - s, cx + s)
		plt.ylim(cy - s, cy + s)

	@property
	def minimumZoomSize(self):
		return 0

	def uniformPointInner(self):
		return self.region.uniformPointInner()

	def intersect(self, other, triedReversed=False):
		return self.region.intersect(other, triedReversed)

	def containsPoint(self, point):
		return self.region.containsPoint(point)

	def containsObject(self, obj):
		return self.region.containsObject(obj)

	def getAABB(self):
		return self.region.getAABB()

	def __str__(self):
		return f'<Workspace on {self.region}>'

	def __eq__(self, other):
		if type(other) is not Workspace:
			return NotImplemented
		return other.region == self.region

	def __hash__(self):
		return hash(self.region)
