"""Advanced RoboSuite Test: Probabilistic Tower Collapse Scenario
Tests bidirectional data flow with physics-based constraints and temporal logic.
"""

model scenic.simulators.robosuite.model

# Probabilistic object types
class Tower(Object):
    width: Range(0.08, 0.15)
    length: Range(0.08, 0.15) 
    height: Range(0.2, 0.5)
    color: Uniform((1, 0, 0), (0, 1, 0), (0, 0, 1))

class Ball(Object):
    width: Range(0.06, 0.12)
    length: self.width
    height: self.width
    color: (1, 1, 0)

# Fixed towers with probabilistic properties
tower1 = new Tower at (Range(-0.3, -0.1), Range(-0.2, 0.2), Range(0.3, 0.8))
tower2 = new Tower at (Range(-0.1, 0.1), Range(-0.2, 0.2), Range(0.3, 0.8))
tower3 = new Tower at (Range(0.1, 0.3), Range(-0.2, 0.2), Range(0.3, 0.8))

# Ensure separation
require (distance from tower1 to tower2) > 0.15
require (distance from tower2 to tower3) > 0.15
require (distance from tower1 to tower3) > 0.15

# Projectile ball
projectile = new Ball at (Range(-0.4, -0.3), 0, Range(0.5, 1.0))

# Target selection - probabilistic
target_tower = Uniform(tower1, tower2, tower3)
ego = tower1  # Fixed ego object

# Physics-based requirements using bidirectional data flow
require always projectile.position.z > -0.1
require eventually (distance from projectile to target_tower) < 0.15

# Temporal monitoring
require always (target_tower.position.z > 0.05) until (
    (distance from projectile to target_tower) < 0.2
)

# Record bidirectional data
record projectile.position as "ball_trajectory"
record target_tower.position as "tower_position" 
record projectile.velocity as "ball_velocity"
record tower1.position.z as "tower1_height"
record tower2.position.z as "tower2_height"  
record tower3.position.z as "tower3_height"

# Stability requirements
require always tower1.position.z > (tower1.height * 0.3)
require always tower2.position.z > (tower2.height * 0.3)
require always tower3.position.z > (tower3.height * 0.3)

# Hard requirement - temporal operators can't be probabilistic
require eventually projectile.position.z < 0.1