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

# make output dirs
os.makedirs(outputDirectory, exist_ok=False)
os.makedirs(asset_dir, exist_ok=False)


assetSubsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
actorSubsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
staticmeshsubsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
system_lib = unreal.SystemLibrary()


# get all static mesh actors
actors = actorSubsystem.get_all_level_actors()
actors = unreal.EditorFilterLibrary.by_class(
    actors, unreal.StaticMeshActor.static_class()
)

# get all static mesh assets
assets = asset_registry.get_all_assets()
assets = asset_registry.get_assets_by_class(
    unreal.TopLevelAssetPath("/Script/Engine", "StaticMesh")
)

# export all asset meshes
for i, asset in enumerate(assets):
    assetname = asset.asset_name
    object_path = str(asset.package_name) + "." + str(asset.asset_name)
    loaded_asset = assetSubsystem.load_asset(object_path)

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
