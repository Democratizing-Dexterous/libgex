from libgex.libex12 import Glove
from libgex.utils import search_ports
import time


port = search_ports()[0]
 
glove = Glove('COM38', vis=True) # COM* for Windows, ttyACM* or ttyUSB* for Linux
glove.connect(init=False) # do not torque on glove yet.


while True:
    glove.fk_finger1() # get the thumb tip xyz position in base_link frame (bottom of the palm), unit m
    glove.fk_finger2() # get the index tip xyz position in base_link frame (bottom of the palm), unit m
    glove.fk_finger3() # get the middle tip xyz position in base_link frame (bottom of the palm), unit m
    
    time.sleep(0.1)