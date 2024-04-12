from verifai.features.features import *
from verifai.samplers.feature_sampler import *
from verifai.falsifier import generic_falsifier
from verifai.monitor import specification_monitor
from dotmap import DotMap
from renderer.generator import genImage
from renderer.kittiLib import getLib
import pickle

# Sampling domain

carDomain = Struct({
    'xPos': Box([0, 1]),
    'yPos': Box([0, 1]),
    'carID': Categorical(*np.arange(0,37))
})

space = FeatureSpace({
    'backgroundID': Feature(Categorical(*np.arange(0, 35))),
    #'cars': Feature(carDomain, lengthDomain=DiscreteBox([1, 2])),
    'cars': Feature(Array(carDomain, (2,))),
    'brightness': Feature(Box([0.5, 1])),
    'sharpness': Feature(Box([0, 1])),
    'contrast': Feature(Box([0.5, 1.5])),
    'color': Feature(Box([0, 1]))
})
sampler = FeatureSampler.randomSamplerFor(space)


class confidence_spec(specification_monitor):
    def __init__(self):
        def specification(traj):
            return bool(traj['yTrue'] == traj['yPred'])
        super().__init__(specification)


MAX_ITERS = 40
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

falsifier_params = DotMap(n_iters=MAX_ITERS,
                          compute_error_table=True,
                          fal_thres=0.5,
                          verbosity=1)

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

falsifier = generic_falsifier(sampler=sampler, server_options=server_options,
                             monitor=confidence_spec(), falsifier_params=falsifier_params)
falsifier.run_falsifier()

analysis_params = DotMap()
analysis_params.k_closest_params.k = 4
analysis_params.random_params.count = 4
analysis_params.pca = True
analysis_params.k_clusters_params.k = 4
falsifier.analyze_error_table(analysis_params=analysis_params)
lib = getLib()

print("Error table")
print(falsifier.error_table.table)
print("Results of error table analysis")
print("Random samples from error table")
for i, sample in enumerate(falsifier.error_analysis.random_samples):
    print("Random Sample : ", i)
    print(sample)
    img, _ = genImage(lib, sample)
    img.save("counterexample_images/random_"+str(i)+".png")
    img.show()
print("k closest samples from error table")
for i, sample in enumerate(falsifier.error_analysis.k_closest_samples):
    print("Sample : ", i)
    print(sample)
    img, _ = genImage(lib, sample)
    img.save("counterexample_images/kclosest_" + str(i) + ".png")

print("k means clustering centroids from error table")
print("Centroids for the categorical parts of the sample")
print(falsifier.error_analysis.k_clusters.keys())

print("Centroids for the numerical parts of the sample for each discrete cluster")
for k in falsifier.error_analysis.k_clusters.keys():
    print(falsifier.error_analysis.k_clusters[k])

print("PCA analysis")
print("PCA pivot: ", falsifier.error_analysis.pca['pivot'])
print("Directions: ", falsifier.error_analysis.pca['directions'])


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))
