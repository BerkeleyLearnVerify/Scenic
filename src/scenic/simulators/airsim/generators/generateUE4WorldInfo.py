import argparse
import json
import os
from pathlib import Path

import unreal

# get output directory
parser = argparse.ArgumentParser()
parser.add_argument(
    "-o",
    "--outputDirectory",
    type=str,
    help="the directory where the fbx info should be dumped. This should be a directory that doesn't exist.",
    required=True,
)


args = parser.parse_args()

outputDirectory = args.outputDirectory + "/"
asset_dir = outputDirectory + "/assets"
actorInfo_dir = outputDirectory + "/actorInfo.json"
worldCenter_dir = outputDirectory + "/worldCenter.json"

# make output dirs
os.makedirs(outputDirectory, exist_ok=False)
os.makedirs(asset_dir, exist_ok=False)


asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
system_lib = unreal.SystemLibrary()


# get all static mesh actors
# actors = actorSubsystem.get_all_level_actors()

actors = unreal.EditorLevelLibrary.get_all_level_actors()
actors = unreal.EditorFilterLibrary.by_class(
    actors, unreal.StaticMeshActor.static_class()
)

assets = asset_registry.get_assets_by_class("StaticMesh")

# get all static mesh assets


# export all asset meshes
for i, asset in enumerate(assets):
    assetname = asset.asset_name
    object_path = str(asset.package_name) + "." + str(asset.asset_name)
    loaded_asset = unreal.load_asset(object_path)

    task = unreal.AssetExportTask()
    task.object = loaded_asset
    task.filename = asset_dir + "/" + str(assetname) + ".fbx"
    task.automated = True  # skip export options prompt
    task.replace_identical = True
    task.options = unreal.FbxExportOption()

    unreal.Exporter.run_asset_export_task(task)


# get all actor info
actor_info_list = []
for i, actor in enumerate(actors):
    actor_name = actor.get_actor_label()

    static_mesh_component = actor.get_component_by_class(unreal.StaticMeshComponent)
    static_mesh = static_mesh_component.static_mesh

    def vecToTuple(vec):
        return (vec.x, vec.y, vec.z)

    def rotorToTuple(rotor):
        return {"pitch": rotor.pitch, "yaw": rotor.yaw, "roll": rotor.roll}

    scale = vecToTuple(actor.get_actor_scale3d())
    loc = vecToTuple(actor.get_actor_location())
    rot = rotorToTuple(actor.get_actor_rotation())

    actor_info_list.append(
        {
            "name": actor_name,
            "scale": scale,
            "location": loc,
            "rotation": rot,
            "meshName": static_mesh.get_name(),
        }
    )

# export actor info
with open(actorInfo_dir, "w") as outfile:
    json.dump(actor_info_list, outfile, indent=4, sort_keys=True)


worldCenter = None
for actor in unreal.EditorLevelLibrary.get_all_level_actors():
    if actor.get_actor_label() == "PlayerStart":
        worldCenter = actor.get_actor_location()
        break


if worldCenter:
    # export actor info
    with open(worldCenter_dir, "w") as outfile:
        json.dump(
            vecToTuple(worldCenter),
            outfile,
            indent=4,
            sort_keys=True,
        )


print("World Info Generated!")
