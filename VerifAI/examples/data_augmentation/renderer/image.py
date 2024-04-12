"""
Basic primitives for creating new images.
The basic object is characterized by image data, collected from the image file
and any other additional parameters the user wants to provide.
"""

from PIL import Image
from renderer.utils import unit2bbH, scale


class LibImage(object):
    def __init__(self, imData, addDetails=None):
        self.image = imData
        self.addDetails = addDetails


class BackObject(LibImage):
    def __init__(self, imData=None, addDetails=None, imPath=None):
        if imData is None:
            imData = Image.open(imPath).data
        self.boundingBoxes = None
        self.scaling = scale(1., 1.)
        super(BackObject, self).__init__(imData, addDetails)

    def updateSampleBox(self, boxes, scale=None):
        self.boundingBoxes = boxes
        self.homographyH = unit2bbH(self.boundingBoxes)
        if scale is not None:
            self.scaling = scale


class ForeObject(LibImage):
    def __init__(self, imData=None, addDetails=None, imPath=None):
        if imData is None:
            imData = Image.open(imPath).data
        super(ForeObject, self).__init__(imData, addDetails)
