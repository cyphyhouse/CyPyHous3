from enum import Enum


class dtypes(Enum):

    INT = type(int(1))
    FLOAT = type(float(0))
    BOOL = type(True)
    STR = type("lol")




def mtype(i):
    if type(i) == type([]):
        print("here")
        return type(i),mtype(i[0])
    else:
        return type(i)

print(mtype([1,2]))