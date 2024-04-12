"""Example aircraft controller.

This controller simply applies a constant throttle, and uses a proportional
controller for the rudder based on the ground truth cross-track and heading
errors.
"""

throttle = 0.4  # constant throttle of plane
cte_gain = 0.1  # gain for cross-track error
he_gain = 0.05  # gain for heading error

def control(server, lat, lon, psi, cte, heading_err):
    rudder = (cte_gain * cte) + (he_gain * heading_err)
    server.sendCTRL([0.0, 0.0, rudder, throttle])
