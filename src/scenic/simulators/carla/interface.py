"""Support code for the CARLA world model."""

from scenic.simulators.formats.opendrive import OpenDriveWorkspace

class CarlaWorkspace(OpenDriveWorkspace):
    @property
    def minimumZoomSize(self):
        return 100
