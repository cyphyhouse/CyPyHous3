from enum import Enum
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Pose
except ImportError:
    print("maybe you don't have ros")

class dtypes(Enum):

    INT = type(int(1))
    FLOAT = type(float(0))
    BOOL = type(True)
    STR = type("lol")
    try:
        POS = type(Pose())
    except :
        print("maybe you don't have ros installed")







def mtype(i):
    if type(i) == type([]):
        print("here")
        return type(i),mtype(i[0])
    else:
        return type(i)

print(mtype([1,2]))