from abc import ABC, abstractmethod

class Seeker(ABC):

    @abstractmethod
    def seek(self):
        raise Exception("subclass must override seek")
    
    @abstractmethod
    def reset(self):
        raise Exception("subclass must override reset")