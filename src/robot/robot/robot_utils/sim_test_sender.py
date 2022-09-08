from collections import OrderedDict
from typing import Tuple
import sim_cam.sim.packet_pb2 as packet_pb2
import socket

# Código refatorado para testes no simulador

message = packet_pb2.Packet()
cmd = message.cmd.robot_commands.add()



def helper() -> None:
    print("")
    print("### HELPER ###")
    helper = "Input a BLUE ROBOT ID (0-2):"
    print(helper)
    print("")

def send_packet(wheels_speed: Tuple[int, int], id: int, sock: socket.socket) -> None:
    global message, cmd

    cmd.id = id  
    cmd.wheel_left, cmd.wheel_right = wheels_speed
    payload = message.SerializeToString()
    sock.sendall(payload)

if __name__ == "__main__":
    helper()
    stop = (0, 0)

    UDP_IP = ""
    UDP_PORT = 20011

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

    sock.connect((UDP_IP, UDP_PORT))

    id = int(input("ID: "))

    while True:
        packet = input("Bytes (q to exit): ")
        if packet[0] == 'q':
            send_packet(stop, id, sock)
            break
        elif packet[0] == "s":
            send_packet(stop, id, sock)
        else:
            speeds = [int(num) for num in packet.split()]
            send_packet(speeds, id, sock)
    
