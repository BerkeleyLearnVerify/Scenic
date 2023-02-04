from scenic_query import *
import scenic
import time

def runExperiment(scenario, numTest, monolithic_translation=False, shuffle=False, debug=False):
    outputDict = queryLabelSetup(scenario,  smt_file_path='./test_smt_encoding.smt2', \
                                 attributeList = ['position', 'heading'],dataType = 'carla',\
                                 monolithic_translation=monolithic_translation)
    errorBound = {}
    errorBound['x'] = 0 # meters == radius of the error margin ball around x
    errorBound['y'] = 0 # meters == radius of the error margin ball around y
    errorBound['heading'] = 0 # radians = 10 degrees
    # print("sortedDependencyList: ", outputDict['sortedDependencyList'])
    
    generatedScenicLabels = []
    time_measurements = []
    
    count = 0
    num_valid = 0
    label = None
    for i in range(numTest):
        count += 1
        unconditionAllAttributes(scenario)
        # print(".......... sampling a new scene from the scenic program ..........")
        scenic_label, _ = scenario.generateForQuery(maxIterations=2000, verbosity=0)
        # print("label sampled --> convert to scenic label")
        label = convertScenicLabel(scenic_label, shuffle)
        # label = {'EgoCar': {'position': (207.26, 8.72), 'heading': 0.01167797486102029}, 'Vehicles': [{'position': (207.18578761887326, 15.074612713734233), 'heading': 0.01167797486102029}, {'position': (204.21070731015672, 15.039868221237322), 'heading': 0.011677974861024953}, {'position': (210.60496379589102, 15.114543582383375), 'heading': 0.07757912446900533}, {'position': (207.10362890820835, 22.109648908620958), 'heading': 0.01167797486102029}, {'position': (204.29256331637814, 22.076819862938557), 'heading': 0.011677974861024953}, {'position': (210.5502414825688, 22.14990019339427), 'heading': 0.030285594938917432}], 'Pedestrians': [], 'Objects': []}
        generatedScenicLabels.append(label)
        # print("sampled label: ", label)
        scenic_testing = not shuffle
        
        current_time = time.time()
        objCountMatched, valid = queryLabel(scenario, label, outputDict, errorBound, ego_visibleDistance=200, dataType='carla', debug=debug, \
                          monolithic_translation=monolithic_translation, scenic_testing=scenic_testing)
                
        query_runtime = time.time()-current_time

        print("number of objects: ", len(scenario.objects))
        
        if not valid:
            print("LABEL NOT VALID: ")
            print("LABEL VALID : ", num_valid)
            print("# of tried samples: ", count)
#             return False, generatedScenicLabels, time_measurements
        else:
            num_valid += 1 
            time_measurements.append(query_runtime)
            if num_valid >= 10:
                break
            print("# of tried samples: ", count)
            print("LABEL VALID : ", num_valid)
            print("query_runtime: ", query_runtime)
        
    return count, generatedScenicLabels, time_measurements

scenic_script = "./examples/carla/scalability/scalability3.scenic"
scenario = scenic.scenarioFromFile(scenic_script)

count, generatedScenicLabels, time_measurements = runExperiment(scenario, numTest=100000, monolithic_translation=False, 
                                                                shuffle=True, debug=False)
print("number of Valid labels: ", len(time_measurements))
print("# of tried samples: ", count)
print("time_measurements: ", time_measurements)

avg = sum(time_measurements)/len(time_measurements)
var = sum([math.pow(x-avg,2) for x in time_measurements])/(len(time_measurements)-1)
std = math.sqrt(var)

print("average runtime: ", avg)
print("standard deviation: ", std)