"""Scenic world model for the Genesis simulator."""

from scenic.core.object_types import Object
from scenic.core.vectors import Vector
from scenic.simulators.genesis.simulator import GenesisSimulator

import genesis as gs
from pathlib import Path
import trimesh
import warnings

param show_viewer = True
param precision = "32"
param substeps = 5

class GenesisObject(Object):
    """Base class for Genesis objects in Scenic."""
    # Genesis-specific properties  
    genesis_entity: None        # Will hold actual Genesis entity after creation
    positionOffset: (0,0,0)
    orientationOffset: (0,0,0)
    fixed: False                # Whether the object is fixed/static
    collision: True             # Whether the object has collision
    
    def makeMorph(self):
        """Create and return the Genesis morph for this object.
        
        This method should be implemented by each subclass to define
        how to create the appropriate Genesis morph for that object type.
        
        Returns:
            Genesis morph object
        """
        raise NotImplementedError(f"makeMorph not implemented for {self.__class__.__name__}")

class GenesisMeshObject(GenesisObject):
    """Genesis object with mesh morph."""
    color: (0.5, 0.5, 0.5)

    def makeMorph(self):
        # NB: Genesis is lazy about accessing the mesh
        assert hasattr(simulation(), "tmpMeshDir")
        objFilePath = Path(simulation().tmpMeshDir.name) / f"{hash(self)}.obj"
        trimesh.exchange.export.export_mesh(self.shape.mesh, objFilePath)
        return gs.morphs.Mesh(
            file=objFilePath,
            scale=(self.width, self.length, self.height),
            fixed=self.fixed,
            collision=self.collision,
            batch_fixed_verts=True,
        )

class PandaRobotArm(GenesisObject):
    """Franka Emika Panda robot"""
    # NOTE: Approximated dimensions
    width: 0.35
    length: 0.26
    height: 1.11
    # Base link is bottom center
    positionOffset: (0,0,-0.5*self.height)
    # Facing towards X axis by default
    orientationOffset: (90 deg, 0, 0)

    robot_file: "xml/franka_emika_panda/panda.xml"

    def makeMorph(self):
        """Create a Genesis Robot morph from MJCF/URDF file."""
        # MJCF files define their own physics properties - no need for 'fixed' parameter
        return gs.morphs.MJCF(
            file=self.robot_file,
            batch_fixed_verts=True,
        )

# Set up global parameters with defaults
if 'backend' not in globalParameters:
    warnings.warn("No Genesis backend specified. Defaulting to GPU.")
    backend = gs.gpu
else:
    if globalParameters.backend == "cpu":
        backend = gs.cpu
    elif globalParameters.backend == "gpu":
        backend = gs.gpu
    elif globalParameters.backend == "cuda":
        backend = gs.cuda
    elif globalParameters.backend == "amdgpu":
        backend = gs.amdgpu
    elif globalParameters.backend == "metal":
        backend = gs.metal
    else:
        raise ValueError(f"Invalid Genesis backend specified: {globalParameters.backend}") 

# Create the simulator instance that Scenic will use
simulator GenesisSimulator(
    genesis_options = {
        "backend": backend,
        "show_viewer": globalParameters.show_viewer,
        "precision": str(globalParameters.precision),
        "substeps": globalParameters.substeps,
    }
)
