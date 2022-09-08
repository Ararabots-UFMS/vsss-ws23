from enum import Enum, unique

@unique
class AcceptanceRadiusEnum(Enum):
    SMALL = 3.
    NORMAL = 5.
    LARGE = 7.
    DEFAULT = 10.
    