from enum import Enum

class dtypes(Enum):
    INT = type(int(1))
    FLOAT = type(float(0))
    BOOL = type(True)
    STR = type("lol")


