Here's the documentation for the path resolution behavior:

## XML Path Resolution in CustomObject

### Embedded XML (string in Scenic file)
```python
# Base directory: Scenic file's location
object_xml = """
<mujoco>
  <asset>
    <mesh file="subfolder/mesh.stl"/>  # Relative to .scenic file
    <texture file="textures/tex.png"/>  # Relative to .scenic file
  </asset>
  ...
</mujoco>
"""
obj = new CustomObject with mjcf_xml object_xml
```

### External XML (file path)
```python
# Base directory: XML file's location
obj = new CustomObject with mjcf_xml "subfolder/object.xml"
```

In `subfolder/object.xml`:
```xml
<mujoco>
  <asset>
    <mesh file="mesh.stl"/>        <!-- Relative to subfolder/ -->
    <texture file="../textures/tex.png"/>  <!-- Must go up to parent -->
  </asset>
  ...
</mujoco>
```

**Key difference:** When loading from file, paths are relative to the XML file's directory, not the Scenic file's directory. This follows standard XML/file system conventions where relative paths are resolved from the document's location.

**Recommendation:** Use embedded XML when you want paths relative to your Scenic file, or adjust paths in external XML files to be relative to their own location.
