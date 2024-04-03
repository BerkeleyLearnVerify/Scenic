param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.carla.model

ego = new Car
atm = new ATM visible
advertisement = new Advertisement visible
barrel = new Barrel visible
barrier = new Barrier visible
bench = new Bench visible
bicycle = new Bicycle visible
box = new Box visible
#busstop = new BusStop visible
casev = new Case visible
chair = new Chair visible
cone = new Cone visible
container = new Container visible
creasedbox = new CreasedBox visible
debris = new Debris visible
garbage = new Garbage visible
gnome = new Gnome visible
ironplate = new IronPlate visible
kiosk = new Kiosk visible
mailbox = new Mailbox visible
motorcycle = new Motorcycle visible
npccar = new NPCCar visible
pedestrian = new Pedestrian visible
plantpot = new PlantPot visible
table = new Table visible
trafficwarning = new TrafficWarning visible
trash = new Trash visible
truck = new Truck visible
vendingmachine = new VendingMachine visible

terminate after 4 seconds