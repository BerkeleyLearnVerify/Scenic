"""Workspaces."""

import trimesh
import numpy as np

from scenic.core.distributions import needsSampling
from scenic.core.regions import (Region, everywhere, MeshVolumeRegion, MeshSurfaceRegion,
                                PolygonalRegion)
from scenic.core.geometry import findMinMax
from scenic.core.errors import RuntimeParseError

class Workspace(Region):
    """A :term:`workspace` describing the fixed world of a scenario.

    Args:
        region (Region): The region defining the extent of the workspace
          (default `everywhere`).
    """
    def __init__(self, region=everywhere):
        if needsSampling(region):
            raise RuntimeParseError('workspace region must be fixed')
        super().__init__('workspace', orientation=region.orientation)

        self.region = region

    def show_3d(self, viewer):
        """Render a schematic of the workspace (in 3D) for debugging"""
        if isinstance(self.region, (MeshVolumeRegion, MeshSurfaceRegion, PolygonalRegion)):
            if isinstance(self.region, (MeshVolumeRegion, MeshSurfaceRegion)):
                workspace_mesh = self.region.mesh.copy()
            else:
                workspace_mesh = self.region.footprint.boundFootprint(center_z=self.region.z, height=0.0001).mesh.copy()
            # We can render this workspace as the wireframe of a mesh
            edges = workspace_mesh.face_adjacency_edges[workspace_mesh.face_adjacency_angles > np.radians(0.1)].copy()
            vertices = workspace_mesh.vertices.copy()

            edge_path = trimesh.path.Path3D(**trimesh.path.exchange.misc.edges_to_path(edges, vertices))

            viewer.add_geometry(edge_path)

    def show_2d(self, plt):
        """Render a schematic of the workspace (in 2D) for debugging"""
        try:
            aabb = self.region.getAABB()
        except NotImplementedError:     # unbounded Regions don't support this
            return
        ((xmin, ymin), (xmax, ymax) , _) = aabb
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

    def zoomAround(self, plt, objects, expansion=1):
        """Zoom the schematic around the specified objects"""
        if not objects:
            return
        positions = (self.scenicToSchematicCoords(obj.position) for obj in objects)
        x, y = zip(*positions)
        minx, maxx = findMinMax(x)
        miny, maxy = findMinMax(y)
        sx = expansion * max(self.minimumZoomSize, 2 * (maxx - minx))
        sy = expansion * max(self.minimumZoomSize, 2 * (maxy - miny))
        s = max(sx, sy) / 2.0
        s += max(max(obj.width, obj.length) for obj in objects) # TODO improve
        cx = (maxx + minx) / 2.0
        cy = (maxy + miny) / 2.0
        plt.xlim(cx - s, cx + s)
        plt.ylim(cy - s, cy + s)

    @property
    def minimumZoomSize(self):
        return 0

    def scenicToSchematicCoords(self, coords):
        """Convert Scenic coordinates to those used for schematic rendering."""
        return coords[:2]

    def uniformPointInner(self):
        return self.region.uniformPointInner()

    def intersect(self, other, triedReversed=False):
        return self.region.intersect(other, triedReversed)

    def intersects(self, other):
        return self.region.intersects(other)

    def difference(self, other):
        return self.region.difference(other)

    def union(self, other, triedReversed=False):
        return self.region.union(other, triedReversed)

    def containsPoint(self, point):
        return self.region.containsPoint(point)

    def containsObject(self, obj):
        return self.region.containsObject(obj)

    def distanceTo(self, point):
        return self.region.distanceTo(point)

    def getAABB(self):
        return self.region.getAABB()

    def __repr__(self):
        return f'Workspace({self.region!r})'

    def __eq__(self, other):
        if type(other) is not Workspace:
            return NotImplemented
        return other.region == self.region

    def __hash__(self):
        return hash(self.region)
