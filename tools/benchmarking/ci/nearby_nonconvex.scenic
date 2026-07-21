region = BoxRegion().difference(BoxRegion(dimensions=(0.1, 0.1, 0.1)))
shape = MeshShape(region.mesh)
ego = new Object with shape shape
other = new Object at (Range(0.9, 1.1), 0), with shape shape