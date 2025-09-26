"""Extract robot meshes from RoboSuite."""

import numpy as np
import trimesh
import mujoco
import os
import re
from collections import defaultdict
from typing import Dict, Set, Optional

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import EmptyArena
from robosuite.models.tasks import ManipulationTask


class RobotExtractor:
    """Extract robots from MuJoCo environment."""
    
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
    
    def extract_robot(self) -> Dict[str, Set[int]]:
        """Extract robot components."""
        components = defaultdict(set)
        robot_indices = set()
        processed_geoms = set()
        
        # Skip world body geoms
        for geom_id in range(self.model.ngeom):
            geom_group = self.model.geom_group[geom_id]
            if geom_group != 1:
                continue
            body_id = self.model.geom_bodyid[geom_id]
            if body_id == 0:
                processed_geoms.add(geom_id)
        
        # Build body hierarchy
        body_to_geoms = defaultdict(set)
        body_to_parent = {}
        body_names = {}
        
        for body_id in range(1, self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            body_names[body_id] = body_name
            parent_id = self.model.body_parentid[body_id]
            body_to_parent[body_id] = parent_id
        
        # Map visual geoms to bodies
        for geom_id in range(self.model.ngeom):
            if geom_id in processed_geoms:
                continue
            geom_group = self.model.geom_group[geom_id]
            if geom_group != 1:
                continue
            body_id = self.model.geom_bodyid[geom_id]
            if body_id > 0:
                body_to_geoms[body_id].add(geom_id)
        
        # Process robot bodies
        processed_bodies = set()
        
        for body_id, geom_ids in body_to_geoms.items():
            if not geom_ids or body_id in processed_bodies:
                continue
            
            body_name = body_names[body_id]
            
            robot_patterns = ['robot', 'link', 'gripper', 'finger', 'hand', 'mount', 'base', 'eef', 'arm', 'robotiq']
            if any(pattern in body_name.lower() for pattern in robot_patterns):
                match = re.search(r'robot(\d+)', body_name.lower())
                if match:
                    idx = int(match.group(1))
                    robot_indices.add(idx)
                    components[f'robot_{idx}'].update(geom_ids)
                else:
                    robot_indices.add(0)
                    components['robot_0'].update(geom_ids)
                processed_bodies.add(body_id)
                continue
            
            # Skip infrastructure
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, list(geom_ids)[0]) or ""
            combined = f"{body_name} {geom_name}".lower()
            if any(kw in combined for kw in ['eef_target', 'controller', 'pedestal', 'torso', 'fixed_mount']):
                processed_bodies.add(body_id)
                continue
        
        # Collect mount/base for robots
        for idx in robot_indices:
            robot_key = f'robot_{idx}'
            for body_id, geom_ids in body_to_geoms.items():
                if body_id in processed_bodies:
                    continue
                body_name = body_names[body_id]
                if any(kw in body_name.lower() for kw in ['mount', 'base', 'pedestal', 'torso']):
                    current = body_id
                    for _ in range(10):
                        if current in body_to_parent:
                            current = body_to_parent[current]
                            if 'robot' in body_names.get(current, '').lower():
                                components[robot_key].update(geom_ids)
                                break
                        else:
                            components[robot_key].update(geom_ids)
                            break
        
        print(f"  Robot indices found: {sorted(robot_indices)}")
        print(f"  Total geoms collected: {len(components.get('robot_0', set()))}")
        
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
    
    def export_robot(self, output_path: str):
        """Export the robot mesh."""
        components = self.extract_robot()
        
        if 'robot_0' in components:
            scene = self._extract_geom_ids(components['robot_0'])
            if scene:
                scene.export(output_path)
                bounds = scene.bounds
                dimensions = bounds[1] - bounds[0]
                print(f"  Meshes in scene: {len(scene.geometry)}")
                return dimensions
        else:
            print(f"  Warning: No robot_0 component found")
        return None


class MinimalEnv(ManipulationEnv):
    """Minimal environment for robot extraction."""
    
    def __init__(self, robots="Panda", **kwargs):
        super().__init__(
            robots=robots, 
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


def extract_all_robots():
    """Extract all robot types."""
    robot_types = ["Panda", "UR5e", "Sawyer", "Jaco", "IIWA"]
    output_dir = "robot_meshes"
    os.makedirs(output_dir, exist_ok=True)
    
    dimensions_info = {}
    
    for robot_type in robot_types:
        print(f"\nExtracting {robot_type}...")
        try:
            env = MinimalEnv(robots=robot_type)
            env.reset()
            
            extractor = RobotExtractor(env)
            output_path = os.path.join(output_dir, f"{robot_type}.glb")
            dimensions = extractor.export_robot(output_path)
            
            if dimensions is not None:
                dimensions_info[robot_type] = dimensions.tolist()
                print(f"  ✓ Saved: {output_path}")
                print(f"  Dimensions: {dimensions}")
            
            env.close()
            
        except Exception as e:
            print(f"  ✗ Failed: {e}")
    
    # Write dimensions file
    with open(os.path.join(output_dir, "dimensions.txt"), "w") as f:
        for robot, dims in dimensions_info.items():
            f.write(f"{robot}: {dims}\n")
    
    print(f"\n✓ Extracted {len(dimensions_info)} robots to {output_dir}/")


if __name__ == "__main__":
    extract_all_robots()