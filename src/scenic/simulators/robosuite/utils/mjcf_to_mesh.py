#!/usr/bin/env python3
"""
Standalone MJCF to GLB converter utility.
Useful for testing MJCF XML conversion without launching RoboSuite/MuJoCo.
"""

import xml.etree.ElementTree as ET
import trimesh
import numpy as np
import argparse
import sys
from pathlib import Path


def convert_mjcf(xml_content, output_file=None, base_path="."):
    """
    Convert MJCF XML string to GLB file.
    
    Args:
        xml_content: MJCF XML string
        output_file: Output filename (auto-generated if None)
        base_path: Base directory for resolving relative mesh paths
    
    Returns:
        Trimesh object
    """
    root = ET.fromstring(xml_content)
    
    if output_file is None:
        model_name = root.get('model', 'output')
        output_file = f"{model_name}.glb"
    
    mesh_geom = root.find('.//geom[@mesh]')
    
    if mesh_geom is not None:
        # Handle mesh file
        mesh_name = mesh_geom.get('mesh')
        mesh_elem = root.find(f'.//asset/mesh[@name="{mesh_name}"]')
        if not mesh_elem:
            raise ValueError(f"Mesh asset '{mesh_name}' not found")
            
        mesh_file = mesh_elem.get('file')
        scale = np.array([float(s) for s in mesh_elem.get('scale', '1 1 1').split()])
        
        # Resolve path
        mesh_path = Path(base_path) / mesh_file if not Path(mesh_file).is_absolute() else Path(mesh_file)
        
        if not mesh_path.exists():
            raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
            
        mesh = trimesh.load(str(mesh_path))
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
        
        mesh.apply_scale(scale)
        
    else:
        # Handle primitives
        geoms = root.findall('.//geom[@type]')
        if not geoms:
            raise ValueError("No mesh or primitive geom found")
        
        meshes = []
        for geom in geoms:
            geom_type = geom.get('type')
            size = np.array([float(s) for s in geom.get('size', '0.1 0.1 0.1').split()])
            
            if geom_type == 'box':
                geom_mesh = trimesh.creation.box(extents=size * 2)
            elif geom_type == 'sphere':
                geom_mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)
            elif geom_type == 'cylinder':
                geom_mesh = trimesh.creation.cylinder(radius=size[0], height=size[1] * 2)
            elif geom_type == 'capsule':
                geom_mesh = trimesh.creation.capsule(radius=size[0], height=size[1] * 2)
            else:
                print(f"Warning: Unsupported geom type: {geom_type}")
                continue
            
            pos = np.array([float(p) for p in geom.get('pos', '0 0 0').split()])
            if np.any(pos != 0):
                geom_mesh.apply_translation(pos)
            
            meshes.append(geom_mesh)
        
        if not meshes:
            raise ValueError("No valid geometry found")
            
        mesh = trimesh.util.concatenate(meshes)
    
    mesh.export(output_file)
    
    # Print info
    volume = mesh.volume if mesh.is_volume else 0
    print(f"Exported: {output_file}")
    print(f"  Vertices: {len(mesh.vertices)}")
    print(f"  Faces: {len(mesh.faces)}")
    print(f"  Volume: {volume:.6f}")
    print(f"  Bounds: {mesh.bounds[1] - mesh.bounds[0]}")
    
    return mesh


def main():
    parser = argparse.ArgumentParser(description='Convert MJCF XML to GLB mesh')
    parser.add_argument('xml_file', nargs='?', help='MJCF XML file path')
    parser.add_argument('-o', '--output', help='Output GLB file')
    parser.add_argument('-b', '--base-path', default='.', help='Base path for mesh files')
    parser.add_argument('--test', action='store_true', help='Run test examples')
    
    args = parser.parse_args()
    
    if args.test:
        run_tests()
    elif args.xml_file:
        with open(args.xml_file, 'r') as f:
            xml_content = f.read()
        
        base_path = Path(args.xml_file).parent if not args.base_path else args.base_path
        convert_mjcf(xml_content, args.output, base_path)
    else:
        parser.print_help()


def run_tests():
    """Run test conversions with example XML."""
    
    # Test 1: Box primitive
    box_xml = """
    <mujoco model="test_box">
      <worldbody>
        <body>
          <body name="object">
            <geom name="box_visual" type="box" size="0.05 0.05 0.05" 
                  pos="0 0 0" rgba="1 0 0 1" group="1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    
    # Test 2: Multi-primitive (table)
    table_xml = """
    <mujoco model="test_table">
      <worldbody>
        <body>
          <body name="object">
            <geom name="top" type="box" size="0.4 0.3 0.025" pos="0 0 0"/>
            <geom name="leg1" type="cylinder" pos="0.35 0.25 -0.4" size="0.02 0.4"/>
            <geom name="leg2" type="cylinder" pos="-0.35 0.25 -0.4" size="0.02 0.4"/>
            <geom name="leg3" type="cylinder" pos="0.35 -0.25 -0.4" size="0.02 0.4"/>
            <geom name="leg4" type="cylinder" pos="-0.35 -0.25 -0.4" size="0.02 0.4"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    
    # Test 3: Sphere
    sphere_xml = """
    <mujoco model="test_sphere">
      <worldbody>
        <body>
          <body name="object">
            <geom name="sphere" type="sphere" size="0.05"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    
    print("Running tests...")
    print("\n1. Box Test:")
    convert_mjcf(box_xml, "test_box.glb")
    
    print("\n2. Table Test:")
    convert_mjcf(table_xml, "test_table.glb")
    
    print("\n3. Sphere Test:")
    convert_mjcf(sphere_xml, "test_sphere.glb")
    
    print("\n✓ Tests complete!")


if __name__ == "__main__":
    main()