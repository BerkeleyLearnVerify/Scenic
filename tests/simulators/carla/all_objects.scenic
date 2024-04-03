param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.carla.model

ego = new Car
atm = new ATM
advertisement = new Advertisement
barrel = new Barrel
barrier = new Barrier
bench = new Bench
bicycle = new Bicycle
box = new Box
busstop = new BusStop
casev = new Case
chair = new Chair
cone = new Cone
container = new Container
creasedbox = new CreasedBox
debris = new Debris
garbage = new Garbage
gnome = new Gnome
ironplate = new IronPlate
kiosk = new Kiosk
mailbox = new Mailbox
motorcycle = new Motorcycle
npccar = new NPCCar
pedestrian = new Pedestrian
plantpot = new PlantPot
table = new Table
trafficwarning = new TrafficWarning
trash = new Trash
truck = new Truck
vendingmachine = new VendingMachine

terminate after 1 seconds