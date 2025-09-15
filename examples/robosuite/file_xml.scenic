# examples/robosuite/file_based_objects.scenic
"""Test loading CustomObjects from XML files with mesh and texture assets."""
model scenic.simulators.robosuite.model

# Get the directory of this scenic file for relative paths
param scenic_file_dir = localPath(".")

# Robot at origin
ego = new Panda at (0, 0, 0)

# Use built-in Table objects
table1 = new Table at (0.6, 0, 0.8)
table2 = new Table at (1.2, 0, 0.8)

# Load bread object from XML file
# Note: mjcf_xml can be either an XML string or a file path
bread = new CustomObject at (0.6, 0, 0.85),
    with mjcf_xml "custom_object/bread.xml"

# Add a simple cube for comparison
blue_ball = new Ball at (1.2, 0, 0.85),
    with color (0, 0, 1, 1),
    with radius 0.03