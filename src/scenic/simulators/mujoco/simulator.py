import mujoco
import mujoco.viewer
from scenic.core.simulators import Simulation, Simulator 
import mujoco.viewer
import time
from scipy.spatial.transform import Rotation
from scenic.core.type_support import toOrientation

import math

from scenic.core.vectors import Vector

class MujocoSimulator(Simulator):
  def __init__(self, xml='', actual = False):
    super().__init__()
    self.xml=xml
    self.actual=actual
  
  def createSimulation(self, scene, **kwargs):
    return MujocoSimulation(scene, self.xml, self.actual, **kwargs)

class MujocoSimulation(Simulation):
  '''
  `Simulation` object for Mujoco.
  '''

  def __init__(self, scene, xml='', actual = False, **kwargs):
    self.xml=xml
    self.actual=actual
    self.scene=scene
    
    self.mujocohandle=None
    kwargs.pop('timestep')
    super().__init__(scene, timestep=.001, **kwargs)


  def setup(self):
    super().setup()
    if self.xml != "":
      self.model = mujoco.MjModel.from_xml_string(self.xml)
      self.data = mujoco.MjData(self.model)
    else:
      object_string=''
      self.xml = f"""
      <mujoco>
        <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
        <default>
          <joint armature="0" damping="1" limited="false"/>
          <geom friction="0.5" solimp="0.99 0.99 0.01" solref="0.01 0.5"/>
        </default>
        <option gravity="0 0 -9.8" timestep="{self.timestep}"/>
        <asset>
          <texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2=".6 .8 1" width="256" height="256"/>
          <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512"/>
          <material name="MatPlane" texture="texplane" texrepeat="1 1" texuniform="true"/>

        </asset>
        <worldbody>
          <light pos="0 1 1" dir="0 -1 -1" diffuse="1 1 1"/>
          <geom condim="3" material="MatPlane" name="ground" pos="0 0 0" size="1 1 0.1" type="plane"/>"""
      x=1
      for obj in self.objects:
        object_string+=f"""<body name="{x}" pos="{obj.position[0]} {obj.position[1]} {obj.position[2]}">
        <joint name="{x}\_joint" type="free" damping="0.001"/>
        <geom name="{x}\_geom" quat="{obj.orientation.q[3]} {obj.orientation.q[0]} {obj.orientation.q[1]} {obj.orientation.q[2]}" size="{obj.width} {obj.length} {obj.height}" rgba="{obj.color[0]} {obj.color[1]} {obj.color[2]} {obj.color[3]}" type = "box" density="100"/>
        </body>
         """
        x=x+1
      self.xml+=object_string
      self.xml+= """
        </worldbody>
      </mujoco>
      """

      self.model = mujoco.MjModel.from_xml_string(self.xml)
      self.data = mujoco.MjData(self.model)
      
    self.mujocohandle = mujoco.viewer.launch_passive(self.model, self.data)

  def createObjectInSimulator(self, obj):
    if self.mujocohandle == None: pass 
    else: 
      print("Mujoco does not handle creation of objects after intialization")
      return -1

  def step(self):
    mujoco.mj_step(self.model, self.data)
    self.mujocohandle.sync()
    if self.actual:
      time.sleep(self.timestep)
    
  def getProperties(self, obj, properties):
    j=1
    
    for checker in self.scene.objects:
      if checker == obj:
        # get postion
        x,y,z=self.data.geom(f'''{j}\_geom''').xpos
        position=Vector(x,y,z)

        # get angular velocity and speed
        a,b,c=self.data.qvel[3:6]
        angularVelocity = Vector(a,b,c)
        angularSpeed = math.hypot(*angularVelocity)

        # get velocity and speed
        a,b,c=self.data.qvel[0:3]
        velocity=Vector(a,b,c)
        speed = math.hypot(*velocity)

        cart_orientation=self.data.geom(f'''{j}\_geom''').xmat 
        a,b,c,d,e,f,g,h,i=cart_orientation
        new_mat = [[a,b,c],[d,e,f,],[g,h,i]]
        r = Rotation.from_matrix(new_mat)
        # ori = toOrientation(r)
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(r)
        break
      j=j+1
    values = dict(
      position=position,
      velocity=velocity,
      speed=speed,
      angularSpeed=angularSpeed,
      angularVelocity=angularVelocity,
      yaw=yaw,#x
      pitch=pitch, #x
      roll=roll,#x
    )
    return values


  def destroy(self):
    if self.mujocohandle != None:
      self.mujocohandle.close()
