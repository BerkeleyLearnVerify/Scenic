"""Scenic world model for robotic manipulation scenarios in ManiSkill.

Global Parameters:
    render (bool): Whether to render the simulation in a window. If True, it will open
        a window and display the simulation. If False (default), the simulation runs headless.
    stride (int): Number of simulation steps between action updates. Default is 1.
"""

try:
    from scenic.simulators.maniskill.simulator import ManiSkillSimulator
except ModuleNotFoundError:
    # for convenience when testing without the mani_skill package
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('The "mani-skill" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def ManiSkillSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'mani-skill' package.

        :meta private:
        """
        raise RuntimeError('the "mani-skill" package is required to run simulations '
                           'from this scenario')

param render = False
param stride = 1

simulator ManiSkillSimulator(
    render=bool(globalParameters.render),
    stride=int(globalParameters.stride),
)

class ManiskillObject(Object):
    """Base class for ManiSkill objects.
    
    Properties:
        visualFilename (str or None): Path to a visual mesh file (e.g., .glb, .obj, .usdz).
        visualScale (tuple): Scale factor for the visual mesh as (x, y, z).
        visualOffset (tuple): Offset for the visual mesh as (x, y, z) to match visual and collision meshes.
        kinematic (bool): If True, the object is kinematic (fixed in place).
    """
    visualFilename: None
    visualScale: (1, 1, 1)
    visualOffset: (0, 0, 0)
    width: 1
    length: 1
    height: 1
    color: [0.8, 0.8, 0.8, 1.0]
    kinematic: False

class Ground(ManiskillObject):
    """The default ground plane from ManiSkill.
    
    This creates a large flat ground plane using ManiSkill's built-in ground builder.
    The ground is always kinematic (fixed in place).
    """
    name: "ground"
    shape: BoxShape()
    width: 100
    length: 100
    height: 0.001
    position: (0, 0, -0.0005)
    kinematic: True
    visualFilename: None
    isSimpleGround: True  # Specify to ManiSkill that this is the ground plane

class Robot(Object):
    """Robot object (Panda arm by default).
    
    This class represents a robot in the scene. The robot is not physically generated
    in the scene (skipGeneration=True) but is used to configure the robot's initial state.
    
    Properties:
        uuid (str): Robot type identifier. Default is "panda" for the Franka Emika Panda arm.
        jointAngles (list): Initial joint angles for the robot. If empty, uses a default configuration.
    """
    position: (10, 0, 0)  # Make scenic ignore it for collision checking for now
    name: "actor"
    uuid: "panda"
    jointAngles: []
    skipGeneration: True  # Don't generate this object in the scene

class Camera(OrientedPoint):
    """Camera configuration for the scene.
    
    Cameras are not physical objects but define viewpoints for rendering and observation.
    
    Properties:
        img_width (int): Image width in pixels. Default is 640.
        img_height (int): Image height in pixels. Default is 480.
        fov (float): Field of view in radians. Default is 1.0.
        near (float): Near clipping plane distance. Default is 0.01.
        far (float): Far clipping plane distance. Default is 100.0.
        shader_pack (str): Shader pack to use for rendering. Default is "rt" (ray tracing).
    """
    name: "camera"
    img_width: 640
    img_height: 480
    fov: 1.0
    near: 0.01
    far: 100.0
    shader_pack: "rt"
    skipGeneration: True  # Don't generate this as a physical object
