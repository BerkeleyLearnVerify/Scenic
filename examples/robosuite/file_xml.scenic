# examples/robosuite/file_xml.scenic
"""Test loading CustomObjects from XML files with mesh and texture assets."""
model scenic.simulators.robosuite.model

param scenic_file_dir = localPath(".")

# Robot at origin
ego = new Panda on (0, 0, 0)

# Use built-in Table object
table1 = new Table on arena_floor

# Load bread object from XML file
bread = new CustomObject on table1,
    with mjcf_xml "custom_object/bread.xml",
    with color (0, 1, 1, 1)