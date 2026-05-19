import argparse

from scenic.simulators.isaac.backends.core_51_usd_to_mesh import asset_convert

if __name__ == "__main__":
    from scenic.simulators.isaac.backends import get_backend

    backend = get_backend("core_51")
    kit = backend.get_simulation_app()
    backend.enable_extension("omni.kit.asset_converter")

    parser = argparse.ArgumentParser("Convert OBJ/STL assets to USD")
    parser.add_argument(
        "--folders",
        type=str,
        nargs="+",
        default=None,
        help="List of folders to convert (space seperated).",
    )
    parser.add_argument(
        "--max-models",
        type=int,
        default=50,
        help="If specified, convert up to `max-models` per folder.",
    )
    parser.add_argument(
        "--load-materials",
        action="store_true",
        help="If specified, materials will be loaded from meshes",
    )
    parser.add_argument(
        "--environments",
        type=str,
        nargs="+",
        default=[],
        help="List of .usd filenames (not paths) to treat as environments.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing converted GLTF files instead of reusing them.",
    )

    args, unknown_args = parser.parse_known_args()
    if args.folders is not None:
        asset_convert(args)
    else:
        print("No folders specified via --folders argument, exiting")

    backend.close_simulation_app(kit)
