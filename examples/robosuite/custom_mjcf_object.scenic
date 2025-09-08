# examples/robosuite/custom_mjcf_object.scenic
model scenic.simulators.robosuite.model

# Table
work_table = new Table at (0.6, 0, 0.8)

# Test with explicit visual properties in XML
test_box = new MJCFObject at (0.5, 0, 0.9),
    with mjcf_xml '<geom type="box" size="0.05 0.05 0.05" rgba="1 0 0 1" contype="0" conaffinity="0" group="1"/>',
    with mjcf_name "test_box"

# Standard box that works
standard_box = new Box at (0.6, 0, 0.9),
    with color (0, 0, 1, 1)

# Robot
ego = new Panda at (0, 0, 0)