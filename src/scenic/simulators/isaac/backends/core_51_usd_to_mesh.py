# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import json
import os
from pathlib import Path
import tempfile
from urllib.parse import urlparse

import numpy as np


def remove_ground_plane(file_path, output_path):
    from pxr import Usd

    stage = Usd.Stage.Open(file_path)
    for prim in stage.Traverse():
        if "groundplane" in str(prim.GetPath()).lower():
            prim.SetActive(False)
    stage.GetRootLayer().Export(output_path)


def compute_bbox(prim):
    from pxr import Usd, UsdGeom

    imageable = UsdGeom.Imageable(prim)
    time = Usd.TimeCode.Default()
    bound = imageable.ComputeWorldBound(time, UsdGeom.Tokens.default_)
    return bound.ComputeAlignedBox()


def flatten_usd(file_path, output_path):
    from pxr import Usd

    stage = Usd.Stage.Open(file_path)
    if stage is None:
        raise RuntimeError(f"could not open USD file for flattening: {file_path}")
    flattened_layer = stage.Flatten()
    flattened_layer.Export(output_path)


def decompose_matrix(mat):
    from pxr import Gf

    reversed_ident_mtx = reversed(Gf.Matrix3d())
    translate = mat.ExtractTranslation()
    scale = Gf.Vec3d(*(v.GetLength() for v in mat.ExtractRotationMatrix()))
    mat.Orthonormalize()
    rotate = Gf.Vec3d(*reversed(mat.ExtractRotation().Decompose(*reversed_ident_mtx)))
    return translate, rotate, scale


def compute_bbox(prim):
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    return cache.ComputeWorldBound(prim).ComputeAlignedBox()


def compute_local_mesh_bbox(prim):
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    points = mesh.GetPointsAttr().Get()

    if not points:
        raise RuntimeError(f"mesh prim has no points: {prim.GetPath()}")

    points_np = np.array([[p[0], p[1], p[2]] for p in points], dtype=float)
    local_min = points_np.min(axis=0)
    local_max = points_np.max(axis=0)
    local_center = (local_min + local_max) * 0.5
    local_size = local_max - local_min

    return local_min, local_max, local_center, local_size


def vec3_to_list(vec):
    return [float(vec[0]), float(vec[1]), float(vec[2])]


def get_mesh_info(usd_path, output_path, info_path, open_stage_func=None):
    import omni
    from pxr import Usd, UsdGeom

    if open_stage_func is None:
        from isaacsim.core.utils.stage import open_stage

        open_stage_func = open_stage

    transforms = {}
    open_stage_func(usd_path)
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    mesh_prims = [
        prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh) and prim.IsValid()
    ]
    if not mesh_prims:
        raise RuntimeError(f"no mesh prims found in environment USD: {usd_path}")

    mesh_infos = []
    for count, prim in enumerate(mesh_prims):
        new_name = f"prim_{count}"
        prim_path = prim.GetPath()
        parent_path = prim_path.GetParentPath()
        new_path = parent_path.AppendChild(new_name)

        bbox = compute_bbox(prim)

        world_min = bbox.GetMin()
        world_max = bbox.GetMax()
        world_center = (world_min + world_max) * 0.5
        world_size = world_max - world_min

        local_min, local_max, local_center, local_size = compute_local_mesh_bbox(prim)

        xformable = UsdGeom.Xformable(prim)
        matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        _, rot, scale = decompose_matrix(matrix)

        scale_np = np.abs(np.array(scale, dtype=float))
        local_size_scaled = np.array(local_size, dtype=float) * scale_np

        mesh_infos.append(
            (
                new_name,
                str(prim_path),
                str(new_path),
                world_center,
                rot,
                world_min,
                world_max,
                world_size,
                local_size_scaled,
            )
        )

    for (
        new_name,
        prim_path,
        new_path,
        world_center,
        rot,
        world_min,
        world_max,
        world_size,
        local_size_scaled,
    ) in mesh_infos:
        omni.kit.commands.execute(
            "MovePrim",
            path_from=prim_path,
            path_to=new_path,
        )

        transforms[new_name] = {
            "full_path": prim_path,
            "orientation": np.array(rot).tolist(),
            "position": vec3_to_list(world_center),
            "world_bbox_center": vec3_to_list(world_center),
            "world_bbox_min": vec3_to_list(world_min),
            "world_bbox_max": vec3_to_list(world_max),
            "world_bbox_size": vec3_to_list(world_size),
            # This is used for the dimensions parameter in MeshShape(..., dimensions=...).
            "usd_dimensions": vec3_to_list(local_size_scaled),
        }

    omni.usd.get_context().save_as_stage(output_path)

    with open(info_path, "w") as out_file:
        json.dump(transforms, out_file, indent=2)

    print(f"---Added {info_path}")


def validate_gltf_geometry(gltf_path):
    with open(gltf_path, "r") as in_file:
        gltf = json.load(in_file)

    if gltf.get("meshes"):
        return

    raise RuntimeError(
        f"converted GLTF has no mesh geometry: {gltf_path}. "
        "If this is an environment, make sure the source USD is flattened and "
        "rerun conversion with --overwrite or delete the stale GLTF first."
    )


def converted_dir_for_folder(folder):
    return os.path.join(folder, "_converted")


def convert_environment_usd(
    usd_path,
    gltf_path,
    info_path,
    *,
    load_materials=True,
    overwrite=False,
    backend_name="core_51",
    open_stage_func=None,
):
    gltf_dir = os.path.dirname(gltf_path)
    info_dir = os.path.dirname(info_path)
    if gltf_dir:
        os.makedirs(gltf_dir, exist_ok=True)
    if info_dir:
        os.makedirs(info_dir, exist_ok=True)

    tmp_dir = tempfile.mkdtemp()
    parsed = urlparse(usd_path)
    model_name = Path(parsed.path if parsed.scheme else usd_path).stem or "environment"

    flattened_usd = os.path.join(tmp_dir, f"{model_name}_flattened.usd")
    flatten_usd(usd_path, flattened_usd)

    renamed_usd = os.path.join(tmp_dir, f"{model_name}_renamed.usd")
    get_mesh_info(flattened_usd, renamed_usd, info_path, open_stage_func=open_stage_func)

    if overwrite or not os.path.exists(gltf_path):
        from scenic.simulators.isaac.backends import get_backend

        status = get_backend(backend_name).convert_sync(
            renamed_usd, gltf_path, load_materials=load_materials
        )
        if not status:
            raise RuntimeError(f"failed to convert environment USD to GLTF: {usd_path}")
        print(f"---Added {gltf_path}")

    validate_gltf_geometry(gltf_path)


def asset_convert(args):
    import omni.client

    for folder in args.folders:
        local_asset_output = converted_dir_for_folder(folder)
        omni.client.create_folder(f"{local_asset_output}")

    tmp_dir = tempfile.mkdtemp()
    for folder in args.folders:
        print(f"\nConverting folder {folder}...")

        _, models = omni.client.list(folder)
        for i, entry in enumerate(models):
            if i >= args.max_models:
                print(f"max models ({args.max_models}) reached, exiting conversion")
                break

            model = str(entry.relative_path)
            model_name = os.path.splitext(model)[0]
            model_format = os.path.splitext(model)[1][1:]
            print(
                f"Model: {model}, Model Name: {model_name}, model_format: {model_format}"
            )
            if model_format != "usd":
                continue

            input_model_path = folder + "/" + model
            if model in args.environments:
                flattened_usd = os.path.join(tmp_dir, f"{model_name}_flattened.usd")
                flatten_usd(input_model_path, flattened_usd)
                renamed_usd = os.path.join(tmp_dir, f"{model_name}_renamed.usd")
                info_path = os.path.join(local_asset_output, f"{model_name}_info.json")
                get_mesh_info(flattened_usd, renamed_usd, info_path)
                input_model_path = renamed_usd
            else:
                usd_without_ground = os.path.join(tmp_dir, f"{model}.usd")
                remove_ground_plane(input_model_path, usd_without_ground)
                input_model_path = usd_without_ground

            converted_model_path = os.path.join(
                local_asset_output, f"{model_name}_{model_format}.gltf"
            )
            if args.overwrite or not os.path.exists(converted_model_path):
                from scenic.simulators.isaac.backends import get_backend

                status = get_backend("core_51").convert_sync(
                    input_model_path, converted_model_path, True
                )
                if not status:
                    print(f"ERROR Status is {status}")
                validate_gltf_geometry(converted_model_path)
                print(f"---Added {converted_model_path}")
            else:
                validate_gltf_geometry(converted_model_path)
