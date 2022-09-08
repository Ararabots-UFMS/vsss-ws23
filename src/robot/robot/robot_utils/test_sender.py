from utils.json_handler import JsonHandler
from collections import OrderedDict
import socket

def show_addresses(bluetooths: OrderedDict) -> None:
    for i, (name, bluetooth) in enumerate(bluetooths.items()):
        print("%02d) %20s\t%s" % (i, name, bluetooth))

def solve_mac(argument: str, bluetooths: OrderedDict) -> str:
    if ':' in argument or '-' in argument:
        return argument
    else:
        index = int(argument)
        mac = bluetooths[list(bluetooths.keys())[index]]
        return mac   

def helper() -> None:
    print("")
    print("### HELPER ###")
    helper = "Input a blueetooth mac address, or one of the indexes of the list below"
    print(helper)
    print("")

def send_packet(packet: str, sock: socket.socket) -> None:
    packet_bytes = bytes([int(b) for b in packet.split(' ')])
    sock.send(packet_bytes)
    

if __name__ == "__main__":
    helper()
    stop = '0 0 0'
    bluetooths = OrderedDict(JsonHandler.read("parameters/bluetooth.json"))
    show_addresses(bluetooths)
    arg = input("mac address: ")
    mac_address = solve_mac(arg, bluetooths)
    
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.connect((mac_address, 0x1001))

    while True:
        packet = input("Bytes (q to exit): ")
        if packet[0] == 'q':
            send_packet(stop, sock)
            break
        else:
            send_packet(packet, sock)
    
    sock.close()
    
