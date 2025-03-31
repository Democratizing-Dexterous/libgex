from libgex.libex12 import Glove
from libgex.utils import search_ports
import time


port = search_ports()[0]
 
glove = Glove('/dev/ttyACM1') # COM* for Windows, ttyACM* or ttyUSB* for Linux
glove.connect(init=False) # do not torque on glove yet.

print(glove.fk_finger1()) # get the thumb tip xyz position in base_link frame (bottom of the palm), unit m
