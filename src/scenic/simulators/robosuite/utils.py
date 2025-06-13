"""Utility functions for Scenic-RoboSuite integration.

This module provides helper functions for coordinate transformations,
color conversions, and other common operations.
"""

import numpy as np
from typing import Tuple, Union, List
from scenic.core.vectors import Vector


def scenic_to_rgba(color, alpha=1.0) -> List[float]:
    """Convert Scenic color to RGBA format for RoboSuite."""
    if hasattr(color, '__iter__') and len(color) >= 3:
        return [float(color[0]), float(color[1]), float(color[2]), float(alpha)]
    elif hasattr(color, '__iter__') and len(color) == 4:
        return [float(color[0]), float(color[1]), float(color[2]), float(color[3])]
    else:
        # Default gray color
        return [0.5, 0.5, 0.5, float(alpha)]


def detect_object_shape(obj) -> str:
    """Detect object shape based on class name."""
    class_name = type(obj).__name__.lower()
    
    if any(keyword in class_name for keyword in ['ball', 'sphere']):
        return 'ball'
    elif any(keyword in class_name for keyword in ['cylinder', 'can', 'bottle']):
        return 'cylinder'
    elif any(keyword in class_name for keyword in ['cube']):
        return 'cube'
    else:
        return 'box'  # Default fallback


def get_object_dimensions(obj) -> Tuple[float, float, float]:
    """Extract object dimensions from Scenic object."""
    width = getattr(obj, 'width', 0.05)
    length = getattr(obj, 'length', 0.05)
    height = getattr(obj, 'height', 0.05)
    
    return (float(width), float(length), float(height))


def print_scene_summary(scene):
    """Print a summary of all objects in a Scenic scene."""
    print(f"Scene Summary: {len(scene.objects)} objects")
    print("=" * 50)
    
    for i, obj in enumerate(scene.objects):
        print(f"[{i+1}] {type(obj).__name__}")
        print(f"    Position: ({obj.position.x:.3f}, {obj.position.y:.3f}, {obj.position.z:.3f})")
        
        width, length, height = get_object_dimensions(obj)
        print(f"    Dimensions: {width:.3f} × {length:.3f} × {height:.3f}")
        
        color = getattr(obj, 'color', None)
        if color:
            rgba = scenic_to_rgba(color)
            print(f"    Color: RGBA({rgba[0]:.2f}, {rgba[1]:.2f}, {rgba[2]:.2f}, {rgba[3]:.2f})")
        
        shape = detect_object_shape(obj)
        print(f"    Detected shape: {shape}")
        print()
