"""Extract arena components (walls and floor) from RoboSuite EmptyArena."""

import numpy as np
import trimesh
import mujoco
import json
import os
from collections import defaultdict
from typing import Dict, Set, Optional

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import EmptyArena
from robosuite.models.tasks import ManipulationTask


class ArenaExtractor:
    """Extract arena components from MuJoCo environment."""
    
    def __init__(self, env):
        self.env = env
        self.model = env.sim.model._model
        self.data = env.sim.data._data
        self.mesh_cache = self._cache_meshes()
    
    def _cache_meshes(self) -> Dict[int, trimesh.Trimesh]:
        """Cache all meshes from MuJoCo model."""
        cache = {}
        for mesh_id in range(self.model.nmesh):
            mesh = self._extract_mesh_data(mesh_id)
            if mesh is not None:
                cache[mesh_id] = mesh
        return cache
    
    def _extract_mesh_data(self, mesh_id: int) -> Optional[trimesh.Trimesh]:
        """Extract single mesh from MuJoCo."""
        try:
            vert_start = self.model.mesh_vertadr[mesh_id]
            vert_count = self.model.mesh_vertnum[mesh_id]
            face_start = self.model.mesh_faceadr[mesh_id]
            face_count = self.model.mesh_facenum[mesh_id]
            
            if vert_count == 0 or face_count == 0:
                return None
            
            vertices = self.model.mesh_vert[vert_start:vert_start + vert_count].reshape(-1, 3)
            faces = self.model.mesh_face[face_start:face_start + face_count].reshape(-1, 3)
            
            scale = self.model.mesh_scale[mesh_id]
            vertices = vertices * scale
            
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            mesh.fix_normals()
            return mesh
        except Exception:
            return None
    
    def _create_primitive(self, geom_type: int, size: np.ndarray) -> Optional[trimesh.Trimesh]:
        """Create primitive geometry mesh."""
        try:
            if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                return trimesh.creation.box(extents=size * 2)
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                return trimesh.creation.icosphere(radius=size[0], subdivisions=2)
            elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                return trimesh.creation.cylinder(radius=size[0], height=size[1] * 2)
            elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
                if size[1] < 0.001:
                    return trimesh.creation.icosphere(radius=size[0], subdivisions=2)
                return trimesh.creation.capsule(radius=size[0], height=size[1] * 2)
            elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                return trimesh.creation.box(extents=[10, 10, 0.01])
            else:
                return None
        except:
            return None
    
    def extract_arena(self) -> Dict[str, Set[int]]:
        """Extract arena components."""
        components = defaultdict(set)
        processed_geoms = set()
        
        # Handle world body geoms (walls, floor)
        for geom_id in range(self.model.ngeom):
            geom_group = self.model.geom_group[geom_id]
            if geom_group != 1:
                continue
            
            body_id = self.model.geom_bodyid[geom_id]
            if body_id == 0:  # World body
                geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                geom_type = self.model.geom_type[geom_id]
                
                # Check for walls first
                if any(kw in geom_name.lower() for kw in ['wall', 'boundary', 'barrier', 'corner']):
                    components['walls'].add(geom_id)
                    processed_geoms.add(geom_id)
                elif 'floor' in geom_name.lower():
                    components['floor'].add(geom_id)
                    processed_geoms.add(geom_id)
                elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                    components['floor'].add(geom_id)
                    processed_geoms.add(geom_id)
        
        print(f"  Found arena components: {list(components.keys())}")
        print(f"  Walls: {len(components.get('walls', set()))} geoms")
        print(f"  Floor: {len(components.get('floor', set()))} geoms")
        
        return components
    
    def _extract_geom_ids(self, geom_ids: Set[int]) -> Optional[trimesh.Scene]:
        """Extract specific geometry IDs into a scene."""
        scene = trimesh.Scene()
        
        for geom_id in geom_ids:
            geom_group = self.model.geom_group[geom_id]
            if geom_group != 1:
                continue
            
            geom_type = self.model.geom_type[geom_id]
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            
            mesh = None
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[geom_id]
                mesh = self.mesh_cache.get(mesh_id)
                if mesh:
                    mesh = mesh.copy()
            elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                mesh = trimesh.creation.box(extents=[10, 10, 0.01])
            else:
                geom_size = self.model.geom_size[geom_id].copy()
                mesh = self._create_primitive(geom_type, geom_size)
            
            if mesh:
                transform = np.eye(4)
                transform[:3, :3] = self.data.geom_xmat[geom_id].reshape(3, 3)
                transform[:3, 3] = self.data.geom_xpos[geom_id]
                
                node_name = geom_name if geom_name else f"geom_{geom_id}"
                scene.add_geometry(mesh, node_name=node_name, transform=transform)
        
        return scene if len(scene.geometry) > 0 else None
    
    def export_arena_components(self, output_dir: str = "arena_meshes"):
        """Export arena components with configuration."""
        os.makedirs(output_dir, exist_ok=True)
        
        components = self.extract_arena()
        metadata = {}
        
        for component_name, geom_ids in components.items():
            if not geom_ids:
                continue
            
            scene = self._extract_geom_ids(geom_ids)
            if scene:
                output_path = os.path.join(output_dir, f"{component_name}.glb")
                scene.export(output_path)
                
                bounds = scene.bounds
                center = (bounds[0] + bounds[1]) / 2
                dimensions = bounds[1] - bounds[0]
                
                metadata[component_name] = {
                    'position': center.tolist(),
                    'dimensions': dimensions.tolist(),
                    'bounds': bounds.tolist(),
                    'num_meshes': len(scene.geometry)
                }
                
                print(f"  ✓ Exported {component_name}.glb ({len(scene.geometry)} meshes)")
                print(f"    Position: {center}")
                print(f"    Dimensions: {dimensions}")
        
        config_path = os.path.join(output_dir, "arena_config.json")
        with open(config_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"\n✓ Saved configuration to {config_path}")
        return metadata


class ArenaOnlyEnv(ManipulationEnv):
    """Minimal environment with just arena."""
    
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
        self.mujoco_arena = EmptyArena()
        self.mujoco_arena.set_origin([0, 0, 0])
        
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[]
        )
    
    def reward(self, action):
        return 0


def extract_arena():
    """Extract arena components."""
    print("Extracting arena components...")
    
    env = ArenaOnlyEnv()
    env.reset()
    
    extractor = ArenaExtractor(env)
    metadata = extractor.export_arena_components()
    
    env.close()
    
    print("\n✓ Arena extraction complete")
    return metadata


if __name__ == "__main__":
    extract_arena()