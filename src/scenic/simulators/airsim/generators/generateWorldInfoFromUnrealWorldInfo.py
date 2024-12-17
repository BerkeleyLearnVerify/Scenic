import argparse
import json
import os
import shutil
from warnings import warn

import bpy
import numpy as np
import trimesh

from scenic.core.utils import repairMesh

WORLD_SCALE = 10000
DEFAULT_MESH = trimesh.creation.box((1, 1, 1))

# get args
parser = argparse.ArgumentParser()
parser.add_argument(
    "-o",
    "--outputDirectory",
    type=str,
    help="the directory where the world info should be dumped. This should be a directory that doesn't exist.",
    required=True,
)
parser.add_argument(
    "-i",
    "--inputDirectory",
    type=str,
    help="the directory where the unreal world info resides",
    required=True,
)
args = parser.parse_args()


outputDirectory = args.outputDirectory + "/"
inputDirectory = args.inputDirectory + "/"

# make dirs
try:
    os.makedirs(args.outputDirectory, exist_ok=False)
except:
    raise RuntimeError("output directory already exists")

assetDir = outputDirectory + "assets/"
objectMeshesDir = outputDirectory + "objectMeshes/"
dumpDir = outputDirectory + "dump/"

os.makedirs(assetDir, exist_ok=False)
os.makedirs(dumpDir, exist_ok=False)
os.makedirs(objectMeshesDir, exist_ok=False)


# export asset objects
assetsInputDir = inputDirectory + "assets/"
tmeshes = {}
for filename in os.listdir(assetsInputDir):
    if filename.endswith(".fbx"):
        # make fbx into stl so that trimesh can read it
        name = os.path.splitext(filename)[0].lower()
        filepath = os.path.join(assetsInputDir, filename)
        newFilePath = dumpDir + name + ".stl"

        bpy.ops.import_scene.fbx(filepath=filepath)
        bpy.ops.export_mesh.stl(filepath=newFilePath, use_selection=True)

        # make tmesh
        tmesh = trimesh.load_mesh(newFilePath)

        if len(tmesh.vertices) == 0:
            print("mesh", name, "has no vertices")
            continue

        # fix tmesh for scenic
        if tmesh.body_count > 1:
            tmesh.fix_normals(multibody=True)
        else:
            tmesh.fix_normals()

        try:
            tmesh = repairMesh(tmesh, verbose=True)
        except Exception as e:
            warn(str(e))
            print("could not repair mesh:", name)
            tmesh = DEFAULT_MESH

        # save tmesh

        # scale = np.array([0.01, 0.01, 0.01])
        # matrix = trimesh.transformations.compose_matrix(scale=scale)
        # tmesh.apply_transform(matrix)

        tmeshes[name] = tmesh

        asset_tmesh = tmesh.copy()
        # apply mesh rotation
        rotation_matrix = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
        asset_tmesh.apply_transform(rotation_matrix)

        with open(
            assetDir + name + ".obj",
            "w",
        ) as outfile:
            outfile.write(trimesh.exchange.obj.export_obj(asset_tmesh))

bpy.ops.wm.quit_blender()

# delete dumpDir
shutil.rmtree(dumpDir)

worldInfo = []

# export objectMeshes and save their world info
with open(inputDirectory + "/actorInfo.json") as file:
    actorInfoList = json.load(file)
    for actorInfo in actorInfoList:
        if not (actorInfo["meshName"].lower() in tmeshes):
            continue

        # save scaled tmesh
        tmesh = tmeshes[actorInfo["meshName"].lower()].copy()

        # apply mesh scale
        scale = np.array(actorInfo["scale"])
        matrix = trimesh.transformations.compose_matrix(scale=scale)
        tmesh.apply_transform(matrix)

        # apply mesh rotation
        rotation_matrix = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
        tmesh.apply_transform(rotation_matrix)

        with open(
            objectMeshesDir + actorInfo["name"] + ".obj",
            "w",
        ) as outfile:
            outfile.write(trimesh.exchange.obj.export_obj(tmesh))

        loc = actorInfo["location"]
        rot = actorInfo["rotation"]

        # save actor data to world info
        worldInfo.append(
            dict(
                name=actorInfo["name"],
                position=[
                    loc[0],
                    loc[1],
                    loc[2],
                ],
                orientation=[rot["pitch"], rot["roll"], rot["yaw"]],
            ),
        )


# export the worldinfo.json
with open(outputDirectory + "worldInfo.json", "w") as outfile:
    json.dump(worldInfo, outfile, indent=4)

print("created world info at:", outputDirectory + "worldInfo.json")
