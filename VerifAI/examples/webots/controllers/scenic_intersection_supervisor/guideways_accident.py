
# Hack to synchronize guideways and Webots coordinates for Intersection Crash experiments

import re
import os.path

base = os.path.dirname(__file__)
jsonPath = os.path.join(base, 'IntersectionCrash_EgoView.json')
osmPath = os.path.join(base, 'IntersectionCrashIntersection.osm')

import scenic.simulators.webots.guideways.intersection as gw_int
gw_int.intersectionPath = jsonPath

# figure out coordinate system used by Webots OSM importer
with open(osmPath, 'r') as osm:
	for line in osm:
		if 'bounds' in line:
			minlat = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('minlat'):])[0])
			minlon = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('minlon'):])[0])
			maxlat = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('maxlat'):])[0])
			maxlon = float(re.findall(r'[-+]?\d*\.\d+|\d+', line[line.find('maxlon'):])[0])
gw_int.intersectionOrigin = ((minlon + maxlon) / 2, (minlat + maxlat) / 2)