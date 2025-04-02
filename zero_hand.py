from libgex.libgx11 import Hand
from libgex.utils import search_ports
import time


 
hand = Hand(port="/dev/ttyACM0") # COM* for Windows, ttyACM* or ttyUSB* for Linux
hand.connect()

print(hand.getj())
# set hand zero, call off() first
hand.off()
hand.set_zero_whole_hand()
print(hand.getj())




