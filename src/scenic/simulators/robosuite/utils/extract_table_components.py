"""Extract table components separately for modular composition."""

import numpy as np
import trimesh
import mujoco
import json
import os

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import MultiTableArena
from robosuite.models.tasks import ManipulationTask


class TableEnv(ManipulationEnv):
    """Environment with table for extraction."""
    
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


def extract_table_components():
    """Extract table top and single leg as separate meshes."""
    print("Extracting table components...")
    
    env = TableEnv()
    env.reset()
    
    model = env.sim.model._model
    data = env.sim.data._data
    
    table_top_scene = trimesh.Scene()
    table_leg_mesh = None
    leg_positions = []
    leg_radius = 0.025
    leg_height = 0.775
    table_top_pos = None
    table_top_size = None
    
    for geom_id in range(model.ngeom):
        if model.geom_group[geom_id] != 1:
            continue
        
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        body_id = model.geom_bodyid[geom_id]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        
        if body_id == 0 or any(kw in body_name.lower() for kw in ['robot', 'wall', 'floor', 'link', 'gripper']):
            continue
        
        if 'table' in body_name.lower() or 'table' in geom_name.lower():
            geom_type = model.geom_type[geom_id]
            geom_size = model.geom_size[geom_id].copy()
            geom_pos = data.geom_xpos[geom_id].copy()
            
            is_top = 'top' in geom_name.lower() or (
                geom_type == mujoco.mjtGeom.mjGEOM_BOX and 
                geom_size[2] < 0.03
            )
            is_leg = 'leg' in geom_name.lower() or (
                geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER
            )
            
            if is_top:
                mesh = trimesh.creation.box(extents=geom_size * 2)
                transform = np.eye(4)
                transform[:3, :3] = data.geom_xmat[geom_id].reshape(3, 3)
                transform[:3, 3] = geom_pos
                table_top_scene.add_geometry(mesh, node_name="table_top", transform=transform)
                table_top_pos = geom_pos.tolist()
                table_top_size = (geom_size * 2).tolist()
                print(f"  Found table top at {geom_pos}")
                
            elif is_leg:
                leg_positions.append(geom_pos.tolist())
                print(f"  Found leg at {geom_pos}")
                
                if table_leg_mesh is None:
                    leg_radius = geom_size[0]
                    leg_height = geom_size[1] * 2
                    mesh = trimesh.creation.cylinder(radius=leg_radius, height=leg_height)
                    table_leg_mesh = mesh
    
    # Calculate offset ratio
    offset_ratio = 0.45
    if leg_positions and len(leg_positions) >= 4 and table_top_size:
        avg_x_offset = np.mean([abs(pos[0]) for pos in leg_positions])
        avg_y_offset = np.mean([abs(pos[1]) for pos in leg_positions])
        
        table_width = table_top_size[0] if table_top_size else 0.8
        table_length = table_top_size[1] if table_top_size else 0.8
        
        if table_width > 0 and table_length > 0:
            x_ratio = avg_x_offset / (table_width / 2)
            y_ratio = avg_y_offset / (table_length / 2)
            offset_ratio = round((x_ratio + y_ratio) / 2, 3)
    
    # Export components
    output_dir = "utils/table_components"
    os.makedirs(output_dir, exist_ok=True)
    
    if len(table_top_scene.geometry) > 0:
        top_path = os.path.join(output_dir, "table_top.glb")
        table_top_scene.export(top_path)
        print(f"✓ Exported table top to {top_path}")
    
    if table_leg_mesh:
        leg_scene = trimesh.Scene()
        leg_scene.add_geometry(table_leg_mesh, node_name="table_leg")
        leg_path = os.path.join(output_dir, "table_leg.glb")
        leg_scene.export(leg_path)
        print(f"✓ Exported table leg to {leg_path}")
    
    # Save metadata
    metadata = {
        "table_top": {
            "default_size": table_top_size if table_top_size else [0.8, 0.8, 0.05],
            "position": table_top_pos if table_top_pos else [0, 0, 0.8],
            "extracted": table_top_pos is not None
        },
        "table_leg": {
            "radius": leg_radius,
            "height": leg_height,
            "offset_from_center": offset_ratio,
            "actual_positions": leg_positions,
            "num_legs_found": len(leg_positions)
        },
        "extraction_info": {
            "table_width": table_top_size[0] if table_top_size else 0.8,
            "table_length": table_top_size[1] if table_top_size else 0.8,
            "table_thickness": table_top_size[2] if table_top_size else 0.05,
            "total_height_with_legs": table_top_pos[2] + table_top_size[2]/2 if table_top_pos and table_top_size else 0.825
        }
    }
    
    config_path = os.path.join(output_dir, "table_config.json")
    with open(config_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"✓ Saved configuration to {config_path}")
    
    env.close()
    return metadata


if __name__ == "__main__":
    extract_table_components()