import carla
# setup the world
address='127.0.0.1' 
port=2000 
map = 'Town01'
client =  carla.Client(address, port)
client.load_world(map)
world = client.get_world()
blueprintLib = world.get_blueprint_library()
ego = None

# Extract blueprint
blueprint = blueprintLib.find('vehicle.mercedes-benz.coupe')
# Set up transform

loc = carla.Location(334.684906, 177.690475, 0.100000)
rot = carla.Rotation(0.000000, 89.979103, 0.000000)
transform = carla.Transform(loc, rot)
# Create Carla actor
carlaActor = world.spawn_actor( blueprint, transform) # <-- not always, but frequently this line returns None causing error on our side	