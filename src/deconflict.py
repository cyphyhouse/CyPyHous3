class vec(object):
    def __init__(self, x, y , z):
        self.x = x
        self.y = y
        self.z = z

class seg(object):
    def __init__(self,p0, p1):
        self.p0 = p0
        self.p1 = p1

def minus(u,v):
    return vec(u.x-v.x, u.y-v.y , u.z -v.z)

def plus(u,v):
    return vec(u.x+v.x, u.y+v.y , u.z +v.z)

def dot_prod(u:vec, v:vec):
    return u.x * v.x + u.y * v.y + u.z * v.z

def norm(v):
    import math
    return math.sqrt(dot_prod(v,v))

def d(u,v):
    return norm(minus(u,v))


def dist3D(l1:seg, l2:seg, tol = 0.000001):
    u = (minus(l1.p1, l1.p0))
    v = (minus(l2.p1, l2.p0))
    w = (minus(l1.p0 ,l2.p0))
    a = dot_prod(u,u)
    b = dot_prod(u,v)
    c = dot_prod(v,v)
    d = dot_prod(u,w)
    e = dot_prod(v,w)
    D = a * c  - b * b
    sc, sN, sD = 0, 0 , D
    tc, tN, tD = 0, 0 , D

    if D < tol :
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = b * e - c * d
        tN = a * e - b * d
        if sN < 0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c
    if tN < 0:
        tN = 0.0
        if -d < 0:
            sN = 0
        elif -d > a:
            sN = sD
            sD = a
    elif tN < tD :
        tN = tD
        if - d + b < 0 :
            sN = 0
        elif - d + b > a:
            sN = sD
        else:
            sN = - d + b
            sD = a
    if abs(sN) < tol :
        sc = 0.0
        tc = 0.0
    else:
        sc = sN/sD
        tc = tN/tD

    dP = plus(w, minus(vec(sc * u.x, sc * u.y, sc* u.z), vec(tc * v.x, tc * v.y, tc * v.z)))
    return norm(dP)

a = vec (1,0,0)
b = vec(0,0,0)
c = seg(a,b)
d = vec(0 ,1,0)
e = vec(1,1, 0)
f = seg(d,e)
print(dist3D(c,f))

