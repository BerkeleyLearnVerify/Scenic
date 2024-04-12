"""Generates a library with images in Kitti forma"""

import numpy as np
from renderer.library import Library

# Helper functions that load the current library objects
BACK_ORIENT = -np.pi / 2
FRONT_ORIENT = np.pi / 2

#Paths to car and road image directories
FORE_SPACES_FILE = './renderer/imgSampSpaces.pickle'  # File with foreground spaces info
LIBRARY_PATH = 'renderer/library/'
BACK_PATH = LIBRARY_PATH + 'roads/'
FORE_PATH = LIBRARY_PATH + 'cars/'


def loadImages():
    """Load car and road images."""

    roadImages = []
    roadImages.append({'roadPath':BACK_PATH + 'desert_kitti.png', \
                   'roadType':'Desert Road', 'roadId':0, 'backgroundColor': 'brown light, blue light', 'environment': 'desert'})
    roadImages.append({'roadPath':BACK_PATH + 'city_kitti.png',\
                   'roadType':'City Road', 'roadId':1, 'backgroundColor': 'brown light, gray', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'forest_kitti.png',\
                   'roadType':'Forest Road', 'roadId':2, 'backgroundColor': 'green light, green dark', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'big_sur_kitti.png',\
                   'roadType':'Big Sur Road', 'roadId':3, 'backgroundColor': 'brown, blue', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'mountain_kitti.jpg',\
                   'roadType':'Mountain Road', 'roadId':4, 'backgroundColor': 'green', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'bridge_kitti.jpg',\
                   'roadType':'Bridge Road', 'roadId':5, 'backgroundColor': 'green, red', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'tunnel_kitti.jpg',\
                   'roadType':'Tunnel Road', 'roadId':6, 'backgroundColor': 'gray', 'environment': 'mountain'})
    roadImages.append({'roadPath':BACK_PATH + 'island_kitti.jpg',\
                   'roadType':'Island Road', 'roadId':7, 'backgroundColor': 'blue light, green, brown light', 'environment': 'field'})
    roadImages.append({'roadPath':BACK_PATH + 'countryside_kitti.jpg',\
                   'roadType':'Countryside Road', 'roadId':8, 'backgroundColor': 'green', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'hill_kitti.jpg',\
                   'roadType':'Hill Road', 'roadId':9, 'backgroundColor': 'green, white', 'environment': 'field'})
    roadImages.append({'roadPath':BACK_PATH + 'alps_kitti.png',\
                   'roadType':'Alps Road', 'roadId':10, 'backgroundColor': 'brown light, gray', 'environment': 'mountain'})
    roadImages.append({'roadPath':BACK_PATH + 'bridge_1_kitti.png',\
                   'roadType':'Bridge 1 Road', 'roadId':11, 'backgroundColor': 'gray light, blue light', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'building_kitti.png',\
                   'roadType':'Building Road', 'roadId':12, 'backgroundColor': 'gray, brown light', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'cloud_kitti.png',\
                   'roadType':'Cloud Road', 'roadId':13, 'backgroundColor': 'green, brown, black', 'environment': 'field'})
    roadImages.append({'roadPath':BACK_PATH + 'downtown_kitti.png',\
                   'roadType':'Downtown Road', 'roadId':14, 'backgroundColor': 'brown light, yellow, gray', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'freeway_kitti.png',\
                   'roadType':'Freeway Road', 'roadId':15, 'backgroundColor': 'gray', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'track_kitti.jpg',\
                   'roadType':'Track Road', 'roadId':16, 'backgroundColor': 'blue, blue light', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'rainforest_kitti.png',\
                   'roadType':'Rainforest Road', 'roadId':17, 'backgroundColor': 'green, brown light', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'tree_kitti.png',\
                   'roadType':'Tree Road', 'roadId':18, 'backgroundColor': 'green, yellow', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'trees_kitti.png',\
                   'roadType':'Trees Road', 'roadId':19, 'backgroundColor': 'green', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'fields_kitti.png',\
                   'roadType':'Fields Road', 'roadId':20, 'backgroundColor': 'green, brown', 'environment': 'forest, fields'})
    roadImages.append({'roadPath':BACK_PATH + 'construction_kitti.png',\
                   'roadType':'Construction Road', 'roadId':21, 'backgroundColor': 'gray, brown', 'environment': 'city'})
    roadImages.append({'roadPath':BACK_PATH + 'little_bridge_kitti.jpg',\
                   'roadType':'Little Bridge', 'roadId':22, 'backgroundColor': 'green, gray', 'environment': 'forest'})
    roadImages.append({'roadPath':BACK_PATH + 'parking_lot_kitti.png',\
                   'roadType':'Parking Lot', 'roadId':23, 'backgroundColor': 'gray', 'environment': 'city, parking'})
    roadImages.append({'roadPath':BACK_PATH + 'indoor_parking_kitti.png',\
                   'roadType':'Indoor Parking Road', 'roadId':24, 'backgroundColor': 'gray', 'environment': 'city, parking'})
    roadImages.append({'roadPath':BACK_PATH + 'freeway_moto_kitti.jpg',\
                   'roadType':'Freeway Moto Road', 'roadId':25, 'backgroundColor': 'black, brow', 'environment': 'desert, freeway'})
    roadImages.append({'roadPath':BACK_PATH + 'freeway_kitti.jpg',\
                   'roadType':'Freeway Road', 'roadId':26, 'backgroundColor': 'black, blue, green', 'environment': 'freeway'})
    roadImages.append({'roadPath':BACK_PATH + 'snow_kitti.jpg',\
                   'roadType':'Snow Road', 'roadId':27, 'backgroundColor': 'white', 'environment': 'snow, forest'})
    roadImages.append({'roadPath':BACK_PATH + 'icy_kitti.jpg',\
                   'roadType':'Icy Road', 'roadId':28, 'backgroundColor': 'white', 'environment': 'snow, forest'})
    roadImages.append({'roadPath':BACK_PATH + 'night_road_kitti.jpg',\
                   'roadType':'Night Road', 'roadId':29, 'backgroundColor': 'black', 'environment': 'fields'})
    roadImages.append({'roadPath':BACK_PATH + 'night_bridge_kitti.jpg',\
                   'roadType':'Night Bridge Road', 'roadId':30, 'backgroundColor': 'black', 'environment': 'bridge'})
    roadImages.append({'roadPath':BACK_PATH + 'in_tunnel_kitti.jpg',\
                   'roadType':'In Tunnel Road', 'roadId':31, 'backgroundColor': 'gray, blue, red', 'environment': 'tunnel'})
    roadImages.append({'roadPath':BACK_PATH + 'rainy_bridge_kitti.jpg',\
                   'roadType':'Rainy Bridge Road', 'roadId':32, 'backgroundColor': 'gray, blue', 'environment': 'bridge'})
    roadImages.append({'roadPath':BACK_PATH + 'joshua_tree_kitti.jpg',\
                   'roadType':'Joshua Tree Road', 'roadId':33, 'backgroundColor': 'brown, green, blue', 'environment': 'desert'})
    roadImages.append({'roadPath':BACK_PATH + 'yosemite_kitti.png',\
                    'roadType':'Yosemite Road', 'roadId':34, 'backgroundColor': 'gray, green, blue', 'environment': 'forest'})


    carImages = [{'carPath':FORE_PATH + 'bmw_gray_front_kitti.png', 'type':'BMW Kitti', \
                   'carId':0, 'carCategory': 'car', 'carColor': 'gray', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'suzuki_rear_kitti.png','type':'Suzuki Kitti',\
                    'carId':1, 'carCategory': 'jeep', 'carColor': 'red dark', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'tesla_rear_kitti.png', 'type':'Tesla Kitti', \
                   'carId':2, 'carCategory': 'car', 'carColor': 'white', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'fiat_front_kitti.png', 'type':'Fiat Kitti',\
                   'carId':3, 'carCategory': 'car', 'carColor': 'green', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'honda_kitti.png', 'type':'Honda Kitti',\
                   'carId':4, 'carCategory': 'car', 'carColor': 'gray', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'toyota_kitti.png', 'type':'Toyota Kitti',\
                   'carId':5, 'carCategory': 'car', 'carColor': 'white', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'peugeot_kitti.png', 'type':'Peugeot Kitti',\
                   'carId':6, 'carCategory': 'car', 'carColor': 'orange', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'chrysler_kitti.png', 'type':'Chrysler Kitti', \
                   'carId':7, 'carCategory': 'van', 'carColor': 'gray', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'bmw_blue_kitti.png', 'type': 'BMW Blue Kitti', \
                   'carId':8, 'carCategory': 'car', 'carColor': 'blue', 'carOrientation': BACK_ORIENT},
                  {'carPath':FORE_PATH + 'honda_civic_front_kitti.png', 'type':'Honda Civic Front Kitti', \
                   'carId':9, 'carCategory': 'car', 'carColor': 'gray', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'toyota_camry_front_kitti.png', 'type': 'Toyota Camry Front Kitti', \
                   'carId':10, 'carCategory': 'car', 'carColor': 'cream', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'toyota_prius_front_kitti.png', 'type': 'Toyota Prius Front Kitti', \
                   'carId':11,  'carCategory': 'car', 'carColor': 'white', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'benz_front_kitti.png', 'type': 'Benz Front Kitti', \
                   'carId':12,  'carCategory': 'car', 'carColor': 'white', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'ford_front_kitti.png', 'type': 'Ford Front Kitti', \
                   'carId':13,  'carCategory': 'car', 'carColor': 'red', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'jeep_front_kitti.png', 'type': 'Jeep Front Kitti', \
                   'carId':14, 'carCategory': 'jeep', 'carColor': 'red', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'jeep_cherokee_front_kitti.png', 'type': 'Jeep Cherokee Front Kitti', \
                   'carId':15, 'carCategory': 'jeep', 'carColor': 'cream', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'fiat_front_kitti.png', 'type': 'Fiat Front Kitti', \
                   'carId':16, 'carCategory': 'car', 'carColor': 'blue', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'bmw_front_kitti.png', 'type': 'BMW Front Kitti', \
                   'carId':17, 'carCategory': 'car', 'carColor': 'blue dark', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'suzuki_front_kitti.png', 'type': 'Suzuki Front Kitti', \
                   'carId':18, 'carCategory': 'jeep', 'carColor': 'red dark', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'volkswagen_golf_front_kitti.png', 'type': 'Volkswagen Golf Kitti', \
                   'carId':19, 'carCategory': 'car', 'carColor': 'blue light', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'toyota_new_prius_front_kitti.png', 'type': 'Toyota New Prius Kitti', \
                   'carId':20, 'carCategory': 'car', 'carColor': 'gray', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'volvo_rear_kitti.png', 'type': 'Volvo Kitti', \
                   'carId':21, 'carCategory': 'car', 'carColor': 'brown', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'porche_rear_kitti.png', 'type': 'Porche Kitti', \
                   'carId':22, 'carCategory': 'car', 'carColor': 'white', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'corvette_front_kitti.png', 'type': 'Corvette Kitti', \
                   'carId':23, 'carCategory': 'car', 'carColor': 'yellow', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'ford_truck_rear_kitti.png', 'type': 'Ford Kitti', \
                   'carId':24, 'carCategory': 'truck', 'carColor': 'white', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'chevrolet_truck_rear_kitti.png', 'type': 'Chevrolet Kitti', \
                   'carId':25, 'carCategory': 'truck', 'carColor': 'red', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'mercedes_rear_kitti.png', 'type': 'Mercedes Kitti', \
                   'carId':26, 'carCategory': 'car', 'carColor': 'black', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'tesla_front_kitti.png', 'type': 'Tesla Kitti', \
                   'carId':27, 'carCategory': 'car', 'carColor': 'black', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'mercedes_front_kitti.png', 'type': 'Mercedes Kitti', \
                   'carId':28, 'carCategory': 'jeep', 'carColor': 'gray', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'mazda_front_kitti.png', 'type': 'Mazda Kitti', \
                   'carId':29, 'carCategory': 'car', 'carColor': 'blue light', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'mazda_rear_kitti.png', 'type': 'Mazda Kitti', \
                   'carId':30, 'carCategory': 'car', 'carColor': 'gray', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'scion_rear_kitti.png', 'type': 'Scion Kitti', \
                   'carId':31, 'carCategory': 'car', 'carColor': 'orange', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'scion_front_kitti.png', 'type': 'Scion Kitti', \
                   'carId':32, 'carCategory': 'car', 'carColor': 'orange', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'fiat_abarth_front_kitti.png', 'type': 'Fiat Abarth Kitti', \
                   'carId':33, 'carCategory': 'car', 'carColor': 'orange', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'volkswagen_beetle_front_kitti.png', 'type': 'Volkswagen Beetle Kitti', \
                   'carId':34, 'carCategory': 'car', 'carColor': 'red dark', 'carOrientation': FRONT_ORIENT },
                  {'carPath': FORE_PATH + 'smart_rear_kitti.png', 'type': 'Smart Kitti', \
                   'carId':35, 'carCategory': 'car', 'carColor': 'black', 'carOrientation': BACK_ORIENT },
                  {'carPath': FORE_PATH + 'smart_front_kitti.png', 'type': 'Smart Kitti', \
                   'carId':36, 'carCategory': 'car', 'carColor': 'blue light', 'carOrientation': FRONT_ORIENT }
                  ]
    return roadImages, carImages


def getLib():
    """Instantiate the library"""
    roadImages, carImages = loadImages()
    return Library(roadImages, carImages, FORE_SPACES_FILE)
