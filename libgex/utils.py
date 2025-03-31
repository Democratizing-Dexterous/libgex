import serial
import serial.tools
import serial.tools.list_ports

def search_ports():
    """
    search for serial ports, COM* for Windows, ttyACM* or ttyUSB* for Linux
    """
    ports = serial.tools.list_ports.comports()

    if len(ports) == 0:
        print('No COM ports found!')
        return None
    
    for p in ports:
        print(p.device, p.serial_number)

    return ports


if __name__ == "__main__":
    ports = search_ports()