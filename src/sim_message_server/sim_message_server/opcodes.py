from enum import Enum

class ServerOpCode(Enum):
    OK            =   0
    ERROR         =   1
    ADD           =   2
    REMOVE        =   3
    ACTIVE        =   4
    INACTIVE      =   5
    CHANGE_COLORS =   6