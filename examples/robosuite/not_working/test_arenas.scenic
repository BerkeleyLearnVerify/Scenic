# test_arenas.scenic
"""Test different arena types"""
model scenic.simulators.robosuite.model

# Example 1: Table Arena
arena = new Table at (0,1,0)

# table2 = new PositionableTable at (0.5, 0, 0),
#     with width 0.6,
#     with length 0.6,
#     with height 0.6

robot = new PandaRobot at (-1, 1, 0)
cube = new Cube at (0, 0, 0.85)

ego = robot


# Example 2: Bins Arena
# arena = BinsArena
# robot = PandaRobot at (0, -0.5, 0.8)
# ego = robot


# Example 3: Custom Arena with XML
# custom_arena = CustomArena with xml_string '''
# <mujoco>
#     <worldbody>
#         <body name="shelf" pos="0 0.5 0.4">
#             <geom type="box" size="0.5 0.05 0.4" rgba="0.7 0.5 0.3 1"/>
#             <geom type="box" pos="0 0 0.4" size="0.5 0.05 0.01" rgba="0.7 0.5 0.3 1"/>
#         </body>
#         <geom name="floor" type="plane" pos="0 0 0" size="3 3 0.125"/>
#     </worldbody>
# </mujoco>
# '''
# robot = PandaRobot at (0, -0.5, 0.8)
# ego = robot


# Example 4: Empty Arena with multiple custom tables
# arena = EmptyArena
# table1 = PositionableTable at (-0.5, 0, 0), with width 0.8, with height 0.8
# table2 = PositionableTable at (0.5, 0, 0), with width 0.6, with height 0.6
# robot = PandaRobot at (0, -0.5, 0.8)
# ego = robot