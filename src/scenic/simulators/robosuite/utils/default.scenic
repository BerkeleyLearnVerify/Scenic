# src/scenic/simulators/robosuite/utils/default.scenic
import trimesh
from scenic.core.utils import repairMesh

# Helper function to load and repair
def load_and_repair(path, name):
    loaded = trimesh.load(path)
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    
    repaired = repairMesh(mesh, pitch=0.02, verbose=False)
    output_path = f"/home/lol/Documents/Scenic/{name}_repaired.glb"
    repaired.export(output_path)
    print(f"{name}: {repaired.bounds[1] - repaired.bounds[0]}")
    return MeshShape(repaired)

# Load and repair meshes
red_cube_shape = load_and_repair("/home/lol/Documents/Scenic/red_cube.glb", "red_cube")
dragon_shape = load_and_repair("/home/lol/Documents/Scenic/dragon.glb", "dragon")
bread_shape = load_and_repair("/home/lol/Documents/Scenic/bread.glb", "bread")
table_shape = load_and_repair("/home/lol/Documents/Scenic/table.glb", "table")

# Load existing table mesh (no repair)
table_mesh = MeshShape.fromFile("/home/lol/Documents/Scenic/src/scenic/simulators/robosuite/utils/table_meshes/standard_table.glb")

# Create objects
standard_table = new Object at (0, 0, 0),
    with shape table_mesh,
    with color (0, 1, 0, 1)

custom_table = new Object at (2, 0, 0),
    with shape table_shape,
    with color (0.6, 0.3, 0.1, 1)

red_cube = new Object at (4, 0, 0),
    with shape red_cube_shape,
    with color (1, 0, 0, 1)

dragon = new Object at (6, 0, 0),
    with shape dragon_shape,
    with color (0.8, 0.2, 0.2, 1)

bread = new Object at (8, 0, 0),
    with shape bread_shape,
    with color (0.9, 0.7, 0.4, 1)