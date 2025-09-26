"""Extract table mesh from RoboSuite's MultiTableArena."""

import numpy as np
import trimesh
import mujoco
import json
import os

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import MultiTableArena
from robosuite.models.tasks import ManipulationTask


class TableEnv(ManipulationEnv):
    """Environment with just a table for extraction."""
    
    def __init__(self, **kwargs):
        super().__init__(
            robots="Panda",
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            **kwargs
        )
    
    def _load_model(self):
        super()._load_model()
        # Create arena with one standard table
        self.mujoco_arena = MultiTableArena(
            table_offsets=[(0, 0, 0.8)],
            table_full_sizes=[(0.8, 0.8, 0.05)],
            has_legs=[True]
        )
        self.mujoco_arena.set_origin([0, 0, 0])
        
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[]
        )
    
    def reward(self, action):
        return 0


def extract_table():
    """Extract table mesh with legs."""
    print("Extracting table mesh...")
    
    env = TableEnv()
    env.reset()
    
    model = env.sim.model._model
    data = env.sim.data._data
    
    # Collect table geometry
    scene = trimesh.Scene()
    
    for geom_id in range(model.ngeom):
        if model.geom_group[geom_id] != 1:  # Only visual
            continue
        
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        body_id = model.geom_bodyid[geom_id]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        
        # Skip robot and arena components
        if any(kw in body_name.lower() for kw in ['robot', 'link', 'gripper', 'wall', 'floor']):
            continue
        if body_id == 0:  # Skip world body
            continue
        
        # Table components
        if 'table' in body_name.lower() or 'table' in geom_name.lower():
            geom_type = model.geom_type[geom_id]
            geom_size = model.geom_size[geom_id].copy()
            
            # Create primitive mesh
            if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                mesh = trimesh.creation.box(extents=geom_size * 2)
            elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                mesh = trimesh.creation.cylinder(radius=geom_size[0], height=geom_size[1] * 2)
            else:
                continue
            
            # Apply transform
            transform = np.eye(4)
            transform[:3, :3] = data.geom_xmat[geom_id].reshape(3, 3)
            transform[:3, 3] = data.geom_xpos[geom_id]
            
            scene.add_geometry(mesh, node_name=f"{body_name}_{geom_name}", transform=transform)
            print(f"  Added: {body_name} - {geom_name}")
    
    # Export
    output_dir = "utils/table_meshes"
    os.makedirs(output_dir, exist_ok=True)
    
    output_path = os.path.join(output_dir, "standard_table.glb")
    scene.export(output_path)
    
    # Calculate dimensions and metadata
    bounds = scene.bounds
    dimensions = bounds[1] - bounds[0]
    center = (bounds[0] + bounds[1]) / 2
    
    metadata = {
        "dimensions": dimensions.tolist(),
        "center": center.tolist(),
        "bounds": bounds.tolist(),
        "table_height": 0.8,  # Standard height
        "table_surface_height": 0.825  # Top surface (0.8 + 0.025)
    }
    
    config_path = os.path.join(output_dir, "table_config.json")
    with open(config_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"✓ Exported table to {output_path}")
    print(f"  Dimensions: {dimensions}")
    print(f"  Center: {center}")
    
    env.close()


if __name__ == "__main__":
    extract_table()