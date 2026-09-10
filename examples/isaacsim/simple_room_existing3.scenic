param environmentUSDPath = localPath("../../assets/Simple_Room/simple_room.usd")

from lib import *
model scenic.simulators.isaac.model
from scenic.simulators.isaac.utils import getExistingObj

simple_room_table = getExistingObj("/Root/table_low_327/table_low")

couch = new Couch above simple_room_table by 1.0,
    facing 0 deg,
    with physics True

dining_table = new DiningTable left of simple_room_table by 0.5,
    facing 0 deg,
    with physics False

floating_toy = new Toy above simple_room_table by 5,
    with physics False
