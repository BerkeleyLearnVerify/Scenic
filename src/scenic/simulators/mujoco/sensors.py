import random
import traceback

import mujoco
import numpy as np

from scenic.core.sensors import RGBSensor, SSSensor


class MujocoRGBSensor(RGBSensor):
    """A simulated RGB camera for MuJoCo that supports efficient data sharing."""

    def __init__(
        self,
        offset=(0, 0, 0),
        rotation=(0, 0, 0),
        width=None,
        height=None,
        attributes=None,
    ):
        if width is None or height is None:
            raise ValueError("Width and height are required for sensors.")

        self.offset = offset
        self.rotation = rotation
        self.width = width
        self.height = height
        self.attributes = attributes or {}

        # These properties are set in setup().
        self.camera_name = None
        self.model = None
        self.data = None
        self.renderer = None

    def getObservation(self):
        """Capture RGB image from the camera."""
        if self.camera_name is None or self.model is None or self.data is None:
            raise RuntimeError("MuJoCo sensor not properly initialized by simulator.")

        if self.renderer is None or not self.renderer.is_initialized():
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

        current_time = self.data.time

        # Check if we can reuse shared RGB data from this timestep.
        if (
            hasattr(self.renderer, "_shared_rgb_time")
            and hasattr(self.renderer, "_shared_rgb_data")
            and self.renderer._shared_rgb_time == current_time
            and self.renderer._shared_rgb_camera == self.camera_name
        ):
            return self.renderer._shared_rgb_data.copy()

        # Otherwise render and cache for sharing.
        try:
            pixels = self.renderer.render(data=self.data, camera_name=self.camera_name)

            if pixels is not None and pixels.size > 0:
                # Cache for sharing with SS sensors.
                self.renderer._shared_rgb_data = pixels
                self.renderer._shared_rgb_time = current_time
                self.renderer._shared_rgb_camera = self.camera_name
                return pixels.copy()
            else:
                return np.zeros((self.height, self.width, 3), dtype=np.uint8)

        except Exception as e:
            print(f"RGB sensor rendering failed: {e}")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)


class MujocoSSSensor(SSSensor):
    """
    Native MuJoCo semantic segmentation sensor using built-in segmentation rendering.
    - Parses geom names (e.g., 'wheel_car_0') to extract semantic classes ('car').
    - Colors objects based on their 'semanticColor' attribute in Scenic.
    - Falls back to a generated color palette for unassigned classes.
    """

    def __init__(
        self,
        offset=None,
        rotation=(0, 0, 0),
        width=None,
        height=None,
        attributes=None,
        use_object_types=False,
        color_map=None,
        color_by_instance=True,
    ):
        if width is None or height is None:
            raise ValueError("width and height are required for sensors")

        self.offset = offset if offset else (0, 0, 0)
        self.rotation = rotation
        self.width = width
        self.height = height
        self.attributes = attributes or {}
        self.use_object_types = use_object_types
        self.color_by_instance = (
            color_by_instance  # Color whole Scenic objects vs individual geoms.
        )

        # Color mapping: dict of keyword -> [R, G, B]
        # If None, uses default palette-based coloring.
        self.color_map = color_map

        # Set by simulator during setup.
        self.camera_name = None
        self.model = None
        self.data = None
        self.renderer = None
        self._last_render_time = -1
        self._class_color_map = {}  # Maps class names to colors.

        # Default color palette for unmapped objects.
        self._color_palette = [
            [255, 0, 0],  # Red
            [0, 255, 0],  # Green
            [0, 0, 255],  # Blue
            [255, 255, 0],  # Yellow
            [255, 0, 255],  # Magenta
            [0, 255, 255],  # Cyan
            [255, 128, 0],  # Orange
            [128, 0, 255],  # Purple
            [255, 128, 128],  # Light Red
            [128, 255, 128],  # Light Green
            [128, 128, 255],  # Light Blue
            [192, 192, 192],  # Silver
        ]

    def getObservation(self):
        """Generate semantic segmentation using MuJoCo's native segmentation rendering."""
        if self.camera_name is None or self.model is None or self.data is None:
            raise RuntimeError("MuJoCo sensor not properly initialized by simulator")

        if self.renderer is None or not self.renderer.is_initialized():
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

        current_time = self.data.time

        # Simple caching to avoid reprocessing.
        if current_time == self._last_render_time and hasattr(
            self, "_cached_segmentation"
        ):
            return self._cached_segmentation.copy()

        try:
            # Create a dedicated segmentation renderer if it doesn't exist.
            if not hasattr(self.renderer, "_seg_renderer"):
                self.renderer._seg_renderer = mujoco.Renderer(
                    self.model, height=self.height, width=self.width
                )
                self.renderer._seg_renderer.enable_segmentation_rendering()

            # Use dedicated segmentation renderer.
            camera_id = -1
            if self.camera_name:
                camera_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
                )
                if camera_id < 0:
                    print(f"WARNING: Camera '{self.camera_name}' not found")

            if camera_id >= 0:
                self.renderer._seg_renderer.update_scene(self.data, camera=camera_id)
            else:
                self.renderer._seg_renderer.update_scene(self.data)

            # Render segmentation.
            seg_data = self.renderer._seg_renderer.render()

            if seg_data is None or seg_data.size == 0:
                return np.zeros((self.height, self.width, 3), dtype=np.uint8)

            # Convert to colored segmentation.
            segmentation = self._segmentation_to_colors(seg_data)

            # Delete seg_data immediately after use to free memory.
            del seg_data

            # Cache the result.
            self._cached_segmentation = segmentation.copy()
            self._last_render_time = current_time

            return segmentation

        except Exception as e:
            print(f"Native segmentation sensor failed: {e}")
            traceback.print_exc()
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def _segmentation_to_colors(self, seg_data):
        """Convert MuJoCo segmentation data to colored visualization."""
        if self.use_object_types:
            ids = seg_data[:, :, 1]  # Use object types.
        else:
            ids = seg_data[:, :, 0]  # Use geom IDs.

        # Handle background.
        ids = ids.astype(np.int32)
        ids[ids == -1] = 0  # Background.

        # Create colored output.
        colored = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Assign colors based on object categories.
        unique_ids = np.unique(ids)
        for obj_id in unique_ids:
            if obj_id == 0:  # Background.
                color = [64, 64, 64]  # Dark gray for background.
            else:
                color = self._get_semantic_color(obj_id)

            mask = ids == obj_id
            colored[mask] = color

        return colored

    def _get_semantic_color(self, geom_id):
        """
        Get semantic color for a geom.

        Priority:
        1. Check if object has semanticColorForGeom method and use it.
        2. Otherwise, use object's semanticColor property.
        3. Fallback to default coloring (color_map or palette).
        """
        if not self.model:
            return [128, 128, 128]  # Gray fallback.

        try:
            # Get geom name.
            if 0 <= geom_id < self.model.ngeom:
                geom_name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                )

                if geom_name:
                    name_lower = geom_name.lower()

                    # Special case: ground/floor always gets gray.
                    if any(word in name_lower for word in ["ground", "floor", "plane"]):
                        return [128, 128, 128]

                    # Try to find the Scenic object for this geom.
                    scenic_obj = None
                    if hasattr(self, "geom_to_object") and self.geom_to_object:
                        scenic_obj = self.geom_to_object.get(geom_id)

                    if scenic_obj:
                        # Priority 1: Check if object has semanticColorForGeom method.
                        if hasattr(scenic_obj, "semanticColorForGeom") and callable(
                            scenic_obj.semanticColorForGeom
                        ):
                            try:
                                geom_color = scenic_obj.semanticColorForGeom(geom_name)
                                if geom_color is not None:
                                    if (
                                        isinstance(geom_color, (list, tuple))
                                        and len(geom_color) == 3
                                    ):
                                        return list(geom_color)
                            except Exception as e:
                                print(
                                    f"Warning: semanticColorForGeom failed for {geom_name}: {e}"
                                )

                        # Priority 2: Use object's semanticColor property.
                        if (
                            hasattr(scenic_obj, "semanticColor")
                            and scenic_obj.semanticColor is not None
                        ):
                            color = scenic_obj.semanticColor
                            if isinstance(color, (list, tuple)) and len(color) == 3:
                                return list(color)

                    # Priority 3: Fallback to class-based coloring.
                    class_name = self._extract_class_name(geom_name)

                    if class_name:
                        # Check if we've already assigned a color to this class.
                        if class_name not in self._class_color_map:
                            # Assign color based on settings.
                            if self.color_by_instance:
                                # Check custom color map first.
                                color_assigned = False
                                if self.color_map:
                                    for keyword, color in self.color_map.items():
                                        if keyword.lower() in class_name.lower():
                                            self._class_color_map[class_name] = color
                                            color_assigned = True
                                            break

                                # Otherwise use palette.
                                if not color_assigned:
                                    available_color = self._get_available_palette_color()
                                    self._class_color_map[class_name] = available_color
                            else:
                                # Color by geom_id (old behavior).
                                color_idx = geom_id % len(self._color_palette)
                                self._class_color_map[class_name] = self._color_palette[
                                    color_idx
                                ]

                        return self._class_color_map[class_name]

                    # No class name found, use geom-based coloring.
                    color_idx = geom_id % len(self._color_palette)
                    return self._color_palette[color_idx]

            return [255, 255, 255]  # White for unknown.

        except Exception as e:
            print(f"Error getting semantic color for geom {geom_id}: {e}")
            return [128, 128, 128]  # Gray fallback.

    def _extract_class_name(self, geom_name):
        """
        Extract class name from geometry name.

        Assumes naming convention: "partname_classname_instancenumber"
        E.g., "wheel_amiga_0" -> "amiga"
              "fl_wheel_amiga_0" -> "amiga"
              "stem_plant_1" -> "plant"
              "rail_left_amiga_0" -> "amiga"
        """
        # Split by underscore.
        parts = geom_name.split("_")

        # Look for pattern where last part is a number.
        # The class name should be the second-to-last part.
        if len(parts) >= 2 and parts[-1].isdigit():
            # Class name is second to last.
            return parts[-2]

        # Fallback: return the full name.
        return geom_name

    def _get_scenic_color_for_class(self, class_name):
        """Check if any Scenic object with this class name has semanticColor defined."""
        # We need access to the geom_to_object mapping.
        if not hasattr(self, "geom_to_object") or not self.geom_to_object:
            return None

        # Look through all mapped objects to find one with this class name.
        for geom_id, scenic_obj in self.geom_to_object.items():
            if hasattr(scenic_obj, "instance_id") and scenic_obj.instance_id:
                # Check if this object's instance_id contains the class_name.
                # E.g., instance_id = "amiga_0", class_name = "amiga"
                if class_name in scenic_obj.instance_id:
                    # Found a matching object, check if it has semanticColor.
                    if (
                        hasattr(scenic_obj, "semanticColor")
                        and scenic_obj.semanticColor is not None
                    ):
                        color = scenic_obj.semanticColor
                        if isinstance(color, (list, tuple)) and len(color) == 3:
                            return list(color)

        return None

    def _get_all_semantic_colors(self):
        """Get all semanticColors that are explicitly defined on Scenic objects."""
        if not hasattr(self, "geom_to_object") or not self.geom_to_object:
            return set()

        semantic_colors = set()
        seen_objects = set()  # Avoid checking same object multiple times.

        for geom_id, scenic_obj in self.geom_to_object.items():
            # Use object id to avoid duplicates.
            obj_id = id(scenic_obj)
            if obj_id in seen_objects:
                continue
            seen_objects.add(obj_id)

            if (
                hasattr(scenic_obj, "semanticColor")
                and scenic_obj.semanticColor is not None
            ):
                color = scenic_obj.semanticColor
                if isinstance(color, (list, tuple)) and len(color) == 3:
                    # Convert to tuple for set storage.
                    semantic_colors.add(tuple(color))

        return semantic_colors

    def _get_available_palette_color(self):
        """
        Get a palette color that doesn't conflict with semanticColors.

        Returns:
            [R, G, B] color list from palette that's not used by semanticColor
        """
        # Get all colors already used by semanticColor.
        reserved_colors = self._get_all_semantic_colors()

        # Also reserve colors already assigned to other classes.
        used_colors = set(tuple(c) for c in self._class_color_map.values())
        reserved_colors.update(used_colors)

        # Find first palette color not in reserved set.
        for color in self._color_palette:
            if tuple(color) not in reserved_colors:
                return color

        # If all palette colors are taken, start generating variations.
        while True:
            # Generate a random color that's not reserved.
            color = [
                random.randint(50, 255),
                random.randint(50, 255),
                random.randint(50, 255),
            ]
            if tuple(color) not in reserved_colors:
                return color


class MujocoRenderer:
    """MuJoCo renderer with shared RGB data caching."""

    def __init__(self, model, width, height):
        self.model = model
        self.width = width
        self.height = height

        # Shared RGB data for sensors using the same camera.
        self._shared_rgb_data = None
        self._shared_rgb_time = -1
        self._shared_rgb_camera = None

        try:
            self.renderer = mujoco.Renderer(model, height=height, width=width)
            self._initialized = True
        except Exception as e:
            print(f"Failed to initialize renderer: {e}")
            self.renderer = None
            self._initialized = False

    def is_initialized(self):
        """
        Check if the underlying MuJoCo renderer context is active and valid.
        """
        return self._initialized and self.renderer is not None

    def render(self, data=None, camera_name=None):
        """Render RGB image."""
        if not self.is_initialized():
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

        try:
            if data is not None:
                camera_id = -1
                if camera_name:
                    camera_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
                    )
                    if camera_id < 0:
                        print(f"WARNING: Camera '{camera_name}' not found")

                if camera_id >= 0:
                    self.renderer.update_scene(data, camera=camera_id)
                else:
                    self.renderer.update_scene(data)

            pixels = self.renderer.render()
            return pixels

        except Exception as e:
            print(f"Rendering failed: {e}")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def close(self):
        """Release MuJoCo rendering context and clear shared caches."""
        # Clean up shared data.
        if hasattr(self, "_shared_rgb_data") and self._shared_rgb_data is not None:
            del self._shared_rgb_data
            self._shared_rgb_data = None

        if hasattr(self, "renderer") and self.renderer:
            try:
                self.renderer.close()
            except Exception as e:
                print(f"WARNING: Error closing renderer: {e}")
            self.renderer = None

        self._initialized = False
