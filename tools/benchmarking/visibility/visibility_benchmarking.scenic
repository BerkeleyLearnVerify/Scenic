param viewRayDensity = 5
param batchSize = 128

workspace = Workspace(everywhere)

import scenic
scenic.core.visibility.BATCH_SIZE = globalParameters.batchSize
