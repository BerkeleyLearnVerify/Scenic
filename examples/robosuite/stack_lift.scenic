# examples/robosuite/stack_lift.scenic
model scenic.simulators.robosuite.model

work_table = new Table on (0.6, 0, 0.8),
    with width 0.6,
    with length 1.2,
    with height 0.05

bottom_cube = new Box on work_table,
    with color (0.2, 0.3, 0.8, 1),
    with width 0.06, with length 0.06, with height 0.06

top_cube = new Box on bottom_cube,
    with color (0.8, 0.2, 0.2, 1)


pickup_object = top_cube

behavior StackLift():
    """Pick up the top cube from the stack and lift it."""
    do PickObject(pickup_object)
    do LiftToHeight(1.05)
    for _ in range(10):
        if pickup_object.position.z > 1.0:
            terminate simulation
        wait
    terminate simulation

ego = new UR5e on (0, 0, 0),
    with behavior StackLift()