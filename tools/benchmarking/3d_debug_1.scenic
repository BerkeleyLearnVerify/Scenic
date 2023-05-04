import trimesh

# Pick a workspace
workspace_region_1 = RectangularRegion(0 @ 0, 0, 20, 20) # Tests 2D Region compatibility
workspace_region_2 = MeshVolumeRegion(trimesh.creation.box((1,1,1)), dimensions=(20,20,20)) # Tests 3D Region compatibility
workspace_region_3 = MeshVolumeRegion(trimesh.creation.icosphere(radius=20)) # Tests 3D Region rejection sampling
workspace_region_4 = MeshVolumeRegion(trimesh.creation.icosphere(radius=20), position=(5,5,5)).intersect(RectangularRegion(0 @ 0, 0, 30, 30)) # Tests intersection of 3D Regions and 2D Regions
workspace_region_5 = MeshVolumeRegion(trimesh.creation.icosphere(radius=20), position=(5,5,5)).difference(RectangularRegion(0 @ 0, 0, 5, 5)) # Tests difference of 3D Regions and 2D Regions
workspace_region_6 = MeshVolumeRegion(trimesh.creation.icosphere(radius=20), position=(5,5,5)).difference(RectangularRegion(0 @ 0, 0, 5, 5).footprint)
workspace_region_7 = workspace_region_6.intersect(RectangularRegion(0 @ 0, 0, 30, 30))
workspace_region_8 = RectangularRegion(0 @ 0, 0, 30, 20).footprint.intersect(CircularRegion(0@0,radius=12.5))

workspace = Workspace(workspace_region_2)

# Place items only in the top or bottom half of workspace, at random
half_sign = Uniform(-1, 1)
sample_space = MeshVolumeRegion(trimesh.creation.box((1,1,1)), dimensions=(20,20,10), position=(0,0,half_sign*5))

# Load chair shape at 1/10th of the original size
chair_shape = MeshShape.fromFile(path=localPath("chair.obj"), type="obj", scale=0.1, initial_rotation=(0,90 deg,0))

# Create the ego object, which will have random dimensions between close to zero and 2
ego = new Object in sample_space,
    with shape BoxShape(dimensions=(Range(0.01,1),Range(0.01,1),Range(0.01,1)), scale=Range(1,2))

# Create a box object in the sample_space
new Object in sample_space,
    with width 1,
    with length 1,
    with height 2,
    with requireVisible False

# Create a chair shaped object in the sample_space.
new Object in sample_space,
    with shape chair_shape,
    with requireVisible False

# Create a chair shaped object with a bounding box
# of size (1,1,1) in the sample_space
new Object in sample_space,
    with width 1,
    with length 1,
    with height 1,
    with shape chair_shape,
    with requireVisible False
