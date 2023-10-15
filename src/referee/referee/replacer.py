from utils.socket_interfaces import SenderSocket
import socket
import struct
# import rospy
from utils.yaml_handler import YamlHandler
from abc import ABC, abstractmethod

import referee.vssref_placement_pb2 as placement_pb2
from referee import vssref_command_pb2

class CMDFactory(ABC):

    def __init__(self, team_color: int,
                       team_side: int,
                       positions: dict):

        self.team_color: int = team_color
        self.team_side: int = team_side # 0 para lado esquerdo e 1 para direito
        self.positions: dict = positions

        self._create_placement_packet = placement_pb2.VSSRef_Placement
    
    def get_positions(self, target_team: int) -> dict:
        # Retorna as posições de reposicionamento, dependendo se o time sofreu ou não a falta.
        if target_team == self.team_color:
            # rospy.logfatal(f"- On DEFENCE")
            return self.positions["DEFENCE"]
        else:
            # rospy.logfatal(f"- On ATTACK")
            return self.positions["ATTACK"]

    @abstractmethod
    def generate_cmd(self, event: vssref_command_pb2.VSSRef_Command) -> str:
        # Método responsável por criar um comando para o Replacer com base no tipo do evento
        # Necessário implementar as correções de lado de campo para cada tipo de evento
        raise NotImplementedError


class PlaceHolderFactory(CMDFactory):

    def __init__(self, team_color: int,
                       team_side: int,
                       positions: dict):

        super().__init__(team_color, team_side, positions)

    def generate_cmd(self, event: vssref_command_pb2.VSSRef_Command):

        packet = self._create_placement_packet()
        frame = packet.world

        frame.teamColor = self.team_color
        # Nao informa nenhuma posição/orientação

        return packet.SerializeToString()


class StandardFactory(CMDFactory):

    '''
    Fábrica de comandos de reposicionamento simples que apenas lê os posicionamentos especificados no arquivo YAML,
    os ajusta dependendo do lado do campo em que a penalidade ocorreu. 
    '''

    def __init__(self, team_color: int,
                       team_side: int,
                       positions: dict):

        super().__init__(team_color, team_side, positions)

    def generate_cmd(self, event: vssref_command_pb2.VSSRef_Command) -> str:

        # Criação do pacote
        packet = self._create_placement_packet()
        # Setando parametros comuns
        frame = packet.world
        frame.teamColor = self.team_color
        # O pacote que chega especifica a cor do time que sofreu a penalidade
        target_team = event.teamcolor

        position_data = self.get_positions(target_team)

        side_correction = (-1) ** self.team_side
        for i in range(3):
            robot = frame.robots.add()
            robot.robot_id = position_data[i]["id"]
            robot.x = side_correction * position_data[i]["x"]
            robot.y = position_data[i]["y"]
            robot.orientation = side_correction * position_data[i]["orientation"]

            # Gambiarra: não da pra espelhar se o angulo é zero :(
            if self.team_side == 1 and (robot.orientation == 0) or (robot.orientation == 45) or (robot.orientation == -45):
                robot.orientation += 180


        return packet.SerializeToString()

class FreeBallFactory(CMDFactory):

    '''
    Implementação da geração de comandos para penalidade de Free Ball.
    '''

    quadrants = {
        1: ( 0.375,  0.4),
        2: (-0.375,  0.4),
        3: (-0.375, -0.4),
        4: ( 0.375, -0.4 )
    }

    def __init__(self, team_color: int,
                       team_side: int,
                       positions: dict):

        super().__init__(team_color, team_side, positions)

    def _is_in_defense_area(self, foul_quadrant: int) -> bool:
        
        if (self.team_side == 0) and ( (foul_quadrant == 2) or (foul_quadrant == 3) ):
            return True
        elif (self.team_side == 1) and ( (foul_quadrant == 1) or (foul_quadrant == 4) ):
            return True
        else:
            return False

    def _place_goalkeeper(self, frame: placement_pb2.VSSRef_Placement.world, foul_quadrant: int) -> None:
        
        side_correction = (-1) ** self.team_side

        robot = frame.robots.add()
        robot.robot_id = 0

        goalkeeper_infos = self.positions[0]
        robot.x = side_correction * goalkeeper_infos["x"]
        robot.y = goalkeeper_infos["y"]
        robot.orientation = goalkeeper_infos["orientation"]

        # Caso o free ball tenha ocorrido no lado de ataque, o goleiro pode se posicionar no centro do gol normalmente
        # Caso contrário, deslocar seu posicionamento em direção ao quadrante do free ball
        if self._is_in_defense_area(foul_quadrant):
            # Quadrantes superiores
            if foul_quadrant < 3:
                robot.y += goalkeeper_infos["offset"]
            # Quadrantes inferiores
            else: 
                robot.y -= goalkeeper_infos["offset"]

    def _place_attacker(self, frame: placement_pb2.VSSRef_Placement.world, foul_quadrant: int) -> None:
        
        side_correction = (-1) ** self.team_side
        free_bal_placement = FreeBallFactory.quadrants[foul_quadrant]

        robot = frame.robots.add()
        robot.robot_id = 2 if not self._is_in_defense_area(foul_quadrant) else 1
        # Gambiarra assumida: devido a mudanças nos comportamentos de zagueiro/atacante, é melhor que eles troquem de posições
        # quando free ball ocorrer no campo de defesa
        robot.x = free_bal_placement[0] - (0.2 * side_correction) # Por o robo obrigatoriamente a 20cm da marcação
        robot.y = free_bal_placement[1]
        robot.orientation = 0

        # Gambiarra: não da pra espelhar se o angulo é zero :(
        if self.team_side == 1 and robot.orientation == 0:
            robot.orientation = 180

    
    def _place_defender(self, frame: placement_pb2.VSSRef_Placement.world, foul_quadrant: int) -> None:
        
        side_correction = (-1) ** self.team_side

        robot = frame.robots.add()

        defender_infos = self.positions[1]
        # Gambiarra assumida: devido a mudanças nos comportamentos de zagueiro/atacante, é melhor que eles troquem de posições
        # quando free ball ocorrer no campo de defesa
        robot.robot_id = 1 if not self._is_in_defense_area(foul_quadrant) else 2
        robot.x = side_correction * defender_infos["x"]
        robot.y = defender_infos["y"]
        robot.orientation = defender_infos["orientation"]

        # Gambiarra: não da pra espelhar se o angulo é zero :(
        if self.team_side == 1 and robot.orientation == 0:
            robot.orientation = 180


        # Caso o free ball tenha ocorrido no lado de ataque, o zagueiro pode se posicionar no centro do campo de defesa
        # Caso contrário, deslocar seu posicionamento na direção oposta ao quadrante do free ball
        if self._is_in_defense_area(foul_quadrant):
            # Quadrantes superiores
            if foul_quadrant < 3:
                robot.y -= defender_infos["offset"]
            # Quadrantes inferiores
            else: 
                robot.y += defender_infos["offset"]
    

    def generate_cmd(self, event: vssref_command_pb2.VSSRef_Command) -> str:

        # Criação do pacote
        packet = self._create_placement_packet()
        # Setando parametros comuns
        frame = packet.world
        frame.teamColor = self.team_color
        # O pacote que chega especifica a cor do time que sofreu a penalidade
        target_team = event.teamcolor
        # Extraindo do pacote, o quandrante onde o evento ocorreu
        foul_quadrant = event.foulQuadrant

        self._place_goalkeeper(frame, foul_quadrant)
        self._place_defender(frame, foul_quadrant)
        self._place_attacker(frame, foul_quadrant)

    
        # TODO: para implementar o free ball é preciso ter acesso a posições dos robos e da bola...

        return packet.SerializeToString()


        
class ReplacerInterface:

    UDP_IP = "224.5.23.2"
    UDP_PORT = 10004

    def __init__(self, 
                 team_side: int,
                 team_color: int,
                 referee_ip = "224.5.23.2", 
                 referee_port = 10003):

        # rospy.logfatal(f"Configurando Replacer para: cor {team_color}, lado {team_side}")
        # self.referee_ip = referee_ip
        # self.referee_port = referee_port

        # self.referee_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.referee_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        # self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        # self.referee_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack("=4sl", socket.inet_aton(self.referee_ip), socket.INADDR_ANY))
        # self.referee_sock.bind((self.referee_ip, self.referee_port))

        self.team_side = team_side
        self.team_color = team_color

        self.all_positions = YamlHandler().read("parameters/replacer_positions.yml")

        self.socket = SenderSocket.create(self.UDP_IP, self.UDP_PORT)
        
        self._setup_factories()

    def _setup_factories(self) -> None:

        self._cmd_factories = {
            # 0: PlaceHolderFactory(self.team_color, self.team_side, None),                      # FREE KICK, não implementado pelo juiz!
            1: StandardFactory(self.team_color, self.team_side, self.all_positions["PENALTY"]),  # PENALTY
            2: StandardFactory(self.team_color, self.team_side, self.all_positions["GOAL_KICK"]),# GOAL KICK
            3: FreeBallFactory(self.team_color, self.team_side, self.all_positions["FREE_BALL"]),# FREEBALL
            4: StandardFactory(self.team_color, self.team_side, self.all_positions["KICKOFF"]),  # KICKOFF 
        }

    def set_team_color_and_side(self, color: int, side: int) -> None:

        self.replacer.team_color = color
        self.replacer.team_side = side


    def handle_event(self, event: vssref_command_pb2.VSSRef_Command) -> None:
        # Método que recebe o evento e o time que sofreu a falta e é responsável por gerar
        # e enviar o comando de reposicionamento para o Referee.
        # Selecionar a fábrica para criação do comando
        factory = self._cmd_factories[event.foul]
        # Geração do comando a ser enviado para o Replacer, baseado no time que sofreu a penalidade
        cmd = factory.generate_cmd(event)
        
        # Envio através do socket
        self.socket.sendall(cmd)