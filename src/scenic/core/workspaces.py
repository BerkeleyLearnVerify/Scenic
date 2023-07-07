"""Workspaces."""

import numpy as np
import trimesh

from scenic.core.distributions import needsSampling
from scenic.core.errors import InvalidScenarioError
from scenic.core.geometry import findMinMax
from scenic.core.regions import (
    MeshSurfaceRegion,
    MeshVolumeRegion,
    PolygonalRegion,
    Region,
    everywhere,
)


class Workspace(Region):
    """A :term:`workspace` describing the fixed world of a scenario.

    Args:
        region (Region): The region defining the extent of the workspace
          (default `everywhere`).
    """

    def __init__(self, region=everywhere):
        if needsSampling(region):
            raise InvalidScenarioError("workspace region must be fixed")
        super().__init__("workspace", orientation=region.orientation)

        self.region = region

    def show3D(self, viewer):
        """Render a schematic of the workspace (in 3D) for debugging"""
        if isinstance(
            self.region, (MeshVolumeRegion, MeshSurfaceRegion, PolygonalRegion)
        ):
            if isinstance(self.region, (MeshVolumeRegion, MeshSurfaceRegion)):
                workspace_mesh = self.region.mesh
            else:
                workspace_mesh = self.region.footprint.boundFootprint(
                    centerZ=self.region.z, height=0.0001
                ).mesh
            # We can render this workspace as the wireframe of a mesh.
            # Filter out meshes that are close enough to planar
            edges = workspace_mesh.face_adjacency_edges[
                workspace_mesh.face_adjacency_angles > np.radians(0.1)
            ]
            vertices = workspace_mesh.vertices

            edge_path = trimesh.path.Path3D(
                **trimesh.path.exchange.misc.edges_to_path(edges, vertices)
            )

            viewer.add_geometry(edge_path)

    def show2D(self, plt):
        """Render a schematic of the workspace (in 2D) for debugging"""
        try:
            aabb = self.region.AABB
        except (NotImplementedError, TypeError):  # unbounded Regions don't support this
            return
        ((xmin, ymin), (xmax, ymax), _) = aabb
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
        s += max(max(obj.width, obj.length) for obj in objects)  # TODO improve
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

    def containsRegionInner(self, reg, tolerance):
        return self.region.containsRegionInner(reg, tolerance)

    def distanceTo(self, point):
        return self.region.distanceTo(point)

    def projectVector(self, point, onDirection):
        raise self.region.projectVector(point, onDirection)

    @property
    def AABB(self):
        return self.region.AABB

    @property
    def dimensionality(self):
        return self.region.dimensionality

    @property
    def size(self):
        return self.region.size

    def __repr__(self):
        return f"Workspace({self.region!r})"

    def __eq__(self, other):
        if type(other) is not Workspace:
            return NotImplemented
        return other.region == self.region

    def __hash__(self):
        return hash(self.region)
