import numpy as np
from dotmap import DotMap

from verifai.client import Client

try:
    import tensorflow as tf
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires tensorflow to be installed')

from renderer.kittiLib import getLib
from renderer.generator import genImage
from model.model import Model


class Classifier(Client):
	def __init__(self, classifier_data):
		port = classifier_data.port
		bufsize = classifier_data.bufsize
		super().__init__(port, bufsize)
		self.sess = tf.compat.v1.Session()
		self.nn = Model()
		self.nn.init(classifier_data.graph_path, classifier_data.checkpoint_path, self.sess)
		self.lib = getLib()

	def simulate(self, sample):
		img, _ = genImage(self.lib, sample)
		yTrue = len(sample.cars)
		yPred = np.argmax(self.nn.predict(np.array(img))[0]) + 1
		res = {}
		res['yTrue'] = yTrue
		res['yPred'] = yPred

		return res



PORT = 8888
BUFSIZE = 4096

classifier_data = DotMap()
classifier_data.port = PORT
classifier_data.bufsize = BUFSIZE
classifier_data.graph_path = './data/car_detector/checkpoint/car-detector-model.meta'
classifier_data.checkpoint_path = './data/car_detector/checkpoint/'

client_task = Classifier(classifier_data)
while True:
	if not client_task.run_client():
		print("End of all classifier calls")
		break
