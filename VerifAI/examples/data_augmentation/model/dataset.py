"""Load and manage a dataset"""

import cv2
import os
import glob
from sklearn.utils import shuffle
import numpy as np


def loadTrain(trainPath, imageSize, classes):
    images = []
    labels = []
    imgNames = []
    cls = []

    print('Going to read training images')
    for fields in classes:
        index = classes.index(fields)
        print('Now going to read {} files (Index: {})'.format(fields, index))
        path = os.path.join(trainPath, fields, '*g')
        files = glob.glob(path)
        for fl in files:
            image = cv2.imread(fl)
            try:
                image = cv2.resize(image, (imageSize, imageSize), 0, 0,
                                   cv2.INTER_LINEAR)
                image = image.astype(np.float32)
                image = np.multiply(image, 1.0 / 255.0)
                images.append(image)
                label = np.zeros(len(classes))
                label[index] = 1.0
                labels.append(label)
                flbase = os.path.basename(fl)
                imgNames.append(flbase)
                cls.append(fields)
            except:
                print('Issue with: ' + fl)
    images = np.array(images)
    labels = np.array(labels)
    imgNames = np.array(imgNames)
    cls = np.array(cls)

    return images, labels, imgNames, cls


class DataSet(object):
    def __init__(self, images, labels, imgNames, cls):
        self._numExamples = images.shape[0]

        self._images = images
        self._labels = labels
        self._imgNames = imgNames
        self._cls = cls
        self._epochsDone = 0
        self._indexInEpoch = 0

    @property
    def images(self):
        return self._images

    @property
    def labels(self):
        return self._labels

    @property
    def imgNames(self):
        return self._imgNames

    @property
    def cls(self):
        return self._cls

    @property
    def num_examples(self):
        return self._numExamples

    @property
    def epochsDone(self):
        return self._epochsDone

    def nextBatch(self, batchSize):
        """Return the next `batchSize` examples from this data set."""
        start = self._indexInEpoch
        self._indexInEpoch += batchSize

        if self._indexInEpoch > self._numExamples:
            # After each epoch we update this
            self._epochsDone += 1
            start = 0
            self._indexInEpoch = batchSize

            assert batchSize <= self._numExamples
        end = self._indexInEpoch

        return self._images[start:end], self._labels[
            start:end], self._imgNames[start:end], self._cls[start:end]


def readTrainSets(trainPath, imageSize, classes, validationSize):
    class DataSets(object):
        pass

    data_sets = DataSets()

    images, labels, imgNames, cls = loadTrain(trainPath, imageSize, classes)
    images, labels, imgNames, cls = shuffle(images, labels, imgNames, cls)

    if isinstance(validationSize, float):
        validationSize = int(validationSize * images.shape[0])

    validationImages = images[:validationSize]
    validationLabels = labels[:validationSize]
    validationImgNames = imgNames[:validationSize]
    validationCls = cls[:validationSize]

    trainImages = images[validationSize:]
    trainLabels = labels[validationSize:]
    trainImgNames = imgNames[validationSize:]
    trainCls = cls[validationSize:]

    data_sets.train = DataSet(trainImages, trainLabels, trainImgNames,
                              trainCls)
    data_sets.valid = DataSet(validationImages, validationLabels,
                              validationImgNames, validationCls)

    return data_sets
