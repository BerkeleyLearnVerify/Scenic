"""Workspaces for the driving domain."""

from scenic.core.workspaces import Workspace

class DrivingWorkspace(Workspace):
    """Workspace created from a road `Network`."""
    def __init__(self, network):
        self.network = network
        super().__init__()

    def show(self, plt):
        self.network.drivableRegion.show(plt)
        self.network.walkableRegion.show(plt, style='b')
        self.network.intersectionRegion.show(plt, style='g')

    @property
    def minimumZoomSize(self):
        return 40
