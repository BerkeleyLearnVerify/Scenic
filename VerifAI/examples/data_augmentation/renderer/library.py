"""Library that contains a list of background and foreground objects"""

import pickle
from renderer.image import *
from renderer.utils import *


class Library():
    """Library of background and foreground objects"""

    def __init__(self, backDB, foreDB, foreSpacesFile=''):

        self.backgroundObjects = []
        self.foregroundObjects = []

        with open(foreSpacesFile, 'rb') as f:
            configs = pickle.load(f)

        for i in range(len(backDB)):
            elem = configs[i]
            imData = Image.open(backDB[i]['roadPath'])
            if elem != []:
                trapezoid = elem[0]
                scaling = elem[1]
                createBound = bb(trapezoid[0], trapezoid[1], trapezoid[2],
                                 trapezoid[3])
                createScale = scale(scaling[0], scaling[1])
                self.addBackgrounds(
                    imData=imData,
                    addDetails=backDB[i],
                    boundingBoxes=createBound,
                    scale=createScale)
            else:
                self.addBackgrounds(imData=imData, addDetails=backInfo[i])

        for car in foreDB:
            imData = Image.open(car['carPath'])
            self.addForegrounds(imData=imData, addDetails=car)

    def addBackgrounds(self, **kwargs):
        if 'backObj' in kwargs:
            self.backgroundObjects.append(kwargs['backObj'])
        else:
            imData = kwargs['imData'] if 'imData' in kwargs else None
            addDetails = kwargs['addDetails'] if 'addDetails' in kwargs \
                    else None
            imPath = kwargs['imPath'] if 'imPath' in kwargs else None
            backObj = BackObject(
                imData=imData, addDetails=addDetails, imPath=imPath)
            self.backgroundObjects.append(backObj)

        scaling = kwargs['scale'] if 'scale' in kwargs else None
        if self.backgroundObjects[-1].boundingBoxes is None:
            if 'boundingBoxes' in kwargs:
                self.backgroundObjects[-1].updateSampleBox(\
                    kwargs['boundingBoxes'], scale=scaling)
            else:
                None
                #print('Please update object with bounding box')

    def addForegrounds(self, **kwargs):
        if 'foreObj' in kwargs:
            self.foregroundObjects.append(kwargs['foreObj'])
        else:
            imData = kwargs['imData'] if 'imData' in kwargs else None
            addDetails = kwargs['addDetails'] if 'addDetails' in kwargs \
                else None
            imPath = kwargs['imPath'] if 'imPath' in kwargs else None
            foreObj = ForeObject(
                imData=imData, addDetails=addDetails, imPath=imPath)
            self.foregroundObjects.append(foreObj)

    def getObj(self, type, id):
        if type == 'background':
            return self.backgroundObjects[id]
        else:
            return self.foregroundObjects[id]
