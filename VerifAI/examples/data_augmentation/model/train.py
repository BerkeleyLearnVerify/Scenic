"""Model trainer"""

from model import dataset, utils
from model.model import Model

import tensorflow as tf
import time
from datetime import timedelta
import math
import random
import numpy as np

# Adding seed so that random initialization is consistent
# from numpy.random import seed
# seed(1)
# from tensorflow import set_random_seed
# set_random_seed(2)

batchSize = 32


# Training paths and model parameters
checkPointName = 'data/checkpoint/car-detector-model'
trainPath = 'data/train/'
classes = ['1', '2']   # training folder names
imgSize = 128
numChannels = 3

# Validation %
validationSize = 0.2


# Load training and validation images and labels

data = dataset.readTrainSets(
    trainPath, imgSize, classes, validationSize=validationSize)

print("Complete reading input data. Will Now print a snippet of it")
print("Number of files in Training-set:\t\t{}".format(len(data.train.labels)))
print("Number of files in Validation-set:\t{}".format(len(data.valid.labels)))

session = tf.Session()

# Get model graph
nn = Model()
x, layerFc2, yTrue, yTrueCls, yPred, yPredCls = nn.getGraph(len(classes))

session.run(tf.global_variables_initializer())

# Training functions
crossEntropy = tf.nn.softmax_cross_entropy_with_logits(
    logits=layerFc2, labels=yTrue)
cost = tf.reduce_mean(crossEntropy)
optimizer = tf.train.AdamOptimizer(learning_rate=1e-4).minimize(cost)
correct_prediction = tf.equal(yPredCls, yTrueCls)
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

session.run(tf.global_variables_initializer())


def showProgress(epoch, feedDictTrain, feedDictValidate, valLoss):
    '''Show progress while training'''
    acc = session.run(accuracy, feed_dict=feedDictTrain)
    val_acc = session.run(accuracy, feed_dict=feedDictValidate)
    msg = "Training Epoch {0} --- Training Accuracy: {1:>6.1%}, Validation Accuracy: {2:>6.1%},  Validation Loss: {3:.3f}"
    print(msg.format(epoch + 1, acc, val_acc, valLoss))


totalIterations = 0

saver = tf.train.Saver()


def train(numIteration):
    '''Training loop'''

    global totalIterations

    for i in range(totalIterations, totalIterations + numIteration):

        # Fecth batch
        xBatch, yTrueBatch, _, _ = data.train.nextBatch(batchSize)
        xValidBatch, yValidBatch, _, _ = data.valid.nextBatch(batchSize)

        feedDictTr = {x: xBatch, yTrue: yTrueBatch}
        feedDictVal = {x: xValidBatch, yTrue: yValidBatch}

        session.run(optimizer, feed_dict=feedDictTr)

        # Show progress and save learnt parameters
        if i % int(data.train.num_examples / batchSize) == 0:
            valLoss = session.run(cost, feed_dict=feedDictVal)
            epoch = int(i / int(data.train.num_examples / batchSize))

            showProgress(epoch, feedDictTr, feedDictVal, valLoss)
            saver.save(session, checkPointName)

    totalIterations += numIteration


train(numIteration=5000)
