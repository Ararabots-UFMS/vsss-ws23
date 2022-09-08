from argparse import ArgumentParser
from typing import Tuple
from utils.watcher import Watcher
import sim_cam.sim.packet_pb2 as packet_pb2
import time
import socket
import matplotlib.pyplot as plt

message = packet_pb2.Environment()

class PrintWatch(Watcher):

    def __init__(self) -> None:
        super().__init__()
    
    def print(self, string: str, period: int = 0.5) -> None:
        '''Imprime uma string desde que a última impressão tenha sido feita a period seconds atrás.'''
        
        cur_time = time.time()

        if cur_time - self.last_time > period:
            print(string)
            self.last_time = cur_time

def get_robot_infos(sock: socket.socket, team: int, id: int) -> Tuple[Tuple[int, int], Tuple[int, int]]:

    data, _ = sock.recvfrom(1024)
    message.ParseFromString(data)

    robots_infos = message.frame.robots_yellow if team == 1 else message.frame.robots_blue
    desired_robot = robots_infos[id]

    vx, vy = desired_robot.vx, desired_robot.vy
    x, y = desired_robot.x, desired_robot.y

    return (vx, vy), (x, y)


if __name__ == "__main__":
    
    parser = ArgumentParser()
    parser.add_argument("--team", type=int, help="Monitor robot from yellow team (0) or blue team (1).")
    parser.add_argument("--id", type=int, help="ID of the robot to monitor.")
    args = parser.parse_args()


    team = args.team
    id = args.id
    # Conectando a porta utilizada pelo Firasim
    UDP_IP = "224.0.0.1"
    UDP_PORT = 10002

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    start_t = time.time()
    acel_t = 0
    v_max = 0

    stopped = True

    v_list = []
    t_list = []
    
    while True:

        try:
            (vx, vy), (x, y) = get_robot_infos(sock, team, id)
            v_abs = (vx*vx + vy*vy)**.5

            print(f"vx: {vx:.4f} // vy: {vy:.4f} // v_abs: {v_abs:.4f} // stopped: {stopped}")

            if v_abs > v_max: v_max = v_abs

        except KeyboardInterrupt:
            print(f"Velocidade máxima atingida {v_max}.")
            break


        # if not stopped:
        #     v_list.append(v_abs)
        #     t_list.append(time.time())

        # if v_abs > 1e-5 and stopped:
        #     start_t = time.time()

        #     v_list = [0]
        #     t_list = [start_t]
        #     stopped = False

        # if v_abs < 1e-5 and not stopped: 
        #     acel_t = time.time() - start_t
        #     stopped = True

        #     break

        # parar qdo cruzar o gol direito
        # if x > 0.75: break
            
    # import pickle
    # save_dict = {
    #     "v": v_list,
    #     "t": t_list
    # }
    # with open("test_with_noise.pkl", "wb") as fout:
    #     pickle.dump(save_dict, fout)

    # plt.plot(t_list, v_list)
    # plt.savefig("speed_test_with_noise.png")



