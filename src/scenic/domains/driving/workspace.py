"""Workspaces for the driving domain."""

from scenic.core.workspaces import Workspace

class DrivingWorkspace(Workspace):
    """Workspace created from a road `Network`."""
    def __init__(self, network):
        self.network = network
        super().__init__()

    def show(self, plt):
        self.network.show()

    @property
    def minimumZoomSize(self):
        return 20
