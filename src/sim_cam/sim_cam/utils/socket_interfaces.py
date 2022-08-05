from abc import ABC, abstractmethod
import socket
#import rospy

class SocketFactory(ABC):

    '''Classe base para criação de interfaces socket.'''

    @classmethod
    def create(cls, UDP_IP: str, UDP_PORT: int) -> socket.socket:
        '''Método reponsável por criar um socket para leitura ou envio de informações.'''
        raise NotImplementedError

class ReceiverSocket(SocketFactory):

    '''Socket para recebimento de dados.'''

    @classmethod
    def create(cls, UDP_IP: str, UDP_PORT: int) -> socket.socket:
        '''Método reponsável por criar um socket para LEITURA de informações.'''
        try:
            sock = socket.socket(socket.AF_INET, # Internet
                                socket.SOCK_DGRAM) # UDP
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((UDP_IP, UDP_PORT))

        except Exception as e:
            print(e)
            pass
            #rospy.logfatal(e)
            #rospy.logfatal("Socket is already running.")

        return sock

class SenderSocket(SocketFactory):

    '''Socket para envio de dados.'''

    @classmethod
    def create(cls, UDP_IP: str, UDP_PORT: int) -> socket.socket:
        '''Método reponsável por criar um socket para ENVIO de informações.'''   

        sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP

        sock.connect((UDP_IP, UDP_PORT))

        return sock


