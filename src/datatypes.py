
def mtype(i):
    if type(i) == type([]):
        print("here")
        return (type(i), mtype(i[0]))
    else:
        return (type(i))


