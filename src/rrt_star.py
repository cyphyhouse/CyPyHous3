"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Amelia Gosse (gossea)
"""

import copy
import math
import random

import numpy as np


class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRT(object):
    """
    Class for RRT* Planning
    """

    def __init__(self):
        pass


    def plan(self,start: list, goal: list, obstacleList: list = [], randArea: list = [-5, 5],
             expandDis: float = 0.55555, goalSampleRate: int = 200, maxIter: int = 500):

        """
        Setting Parameters

        start: Start position [x,y]
        goal: Goal position [x,y]
        obstacleList: Obstacle positions [[x,y,size],...]
        randArea: Random sampling bounds [min,max]

        """
        # self.__motionautomaton = motionautomaton
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, search_until_maxiter: bool = False) -> list:
        """
        RRT* Path Planning
        search_until_maxiter: Search until max iteration for path improving or not
        """
        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            new_node = self.steer(rnd, nind)

            if self.__CollisionCheck(new_node, self.obstacleList):
                nearinds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, nearinds)
                self.nodeList.append(new_node)
                self.rewire(new_node, nearinds)

            # generate course
            if not search_until_maxiter:
                lastIndex = self.get_best_last_index()
                if lastIndex:
                    path = self.gen_final_course(lastIndex)
                    return to_path(path[::-1])

        print("Reached max iteration")

        lastIndex = self.get_best_last_index()
        if lastIndex:
            path = self.gen_final_course(lastIndex)
            return to_path(path[::-1])

        return None

    def choose_parent(self, new_node, nearinds: list):
        if not nearinds:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - self.nodeList[i].x
            dy = new_node.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, rnd: list, nind: int):

        # expand tree
        nearest_node = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            new_node.x = nearest_node.x + self.expandDis * math.cos(theta)
            new_node.y = nearest_node.y + self.expandDis * math.sin(theta)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if not goalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind: int):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x: float, y: float):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, new_node, nearinds: list):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = new_node.x - nearNode.x
            dy = new_node.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta: float, d: float):
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False
        return True


    def GetNearestListIndex(self, nodeList: list, rnd: list):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList: list):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision

        return True  # safe


class vec(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def topoint(self):
        return [self.x, self.y, self.z]

    def magnitude(self):
        import math
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __repr__(self):
        return "("+str(self.x) + "," + str(self.y) + "," + str(self.z)+")"


class seg(object):
    def __init__(self, p0, p1):
        self.p0 = p0
        self.p1 = p1

    def __repr__(self):
        return str(self.p0) + ":" + str(self.p1)


def minus(u, v):
    return vec(u.x - v.x, u.y - v.y, u.z - v.z)


def plus(u, v):
    return vec(u.x + v.x, u.y + v.y, u.z + v.z)


def dot_prod(u: vec, v: vec):
    return u.x * v.x + u.y * v.y + u.z * v.z


def norm(v):
    import math
    return math.sqrt(dot_prod(v, v))


def cross_prod(u: vec, v: vec):
    x1 = u.y * v.z - u.z * v.y
    y1 = u.z * v.x - u.x * v.z
    z1 = u.x * v.y - u.y * v.x
    return vec(x1, y1, z1)


def d(u, v):
    return norm(minus(u, v))


def dist3D(l1: seg, l2: seg, tol=0.000001):
    u = (minus(l1.p1, l1.p0))
    v = (minus(l2.p1, l2.p0))
    w = (minus(l1.p0, l2.p0))
    a = dot_prod(u, u)
    b = dot_prod(u, v)
    c = dot_prod(v, v)
    d = dot_prod(u, w)
    e = dot_prod(v, w)
    D = a * c - b * b
    sc, sN, sD = 0, 0, D
    tc, tN, tD = 0, 0, D

    if D < tol:
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
    elif tN < tD:
        tN = tD
        if - d + b < 0:
            sN = 0
        elif - d + b > a:
            sN = sD
        else:
            sN = - d + b
            sD = a
    if abs(sN) < tol:
        sc = 0.0
        tc = 0.0
    else:
        sc = sN / sD
        tc = tN / tD

    dP = plus(w, minus(vec(sc * u.x, sc * u.y, sc * u.z), vec(tc * v.x, tc * v.y, tc * v.z)))
    return norm(dP)


def collinear(p1: vec, p2: vec, p3: vec):
    n1 = vec(p2.x - p1.x, p2.y - p1.y, p2.z - p3.z)
    n2 = vec(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)
    n3 = cross_prod(n1, n2)
    if n3.magnitude() == 0:
        return True


def DistancePtLine(l: seg, p: vec):
    if not collinear(l.p0, l.p1, p):
        ab = minus(l.p1, l.p0)
        ac = minus(p, l.p0)
        area = cross_prod(ab, ac).magnitude()
        cd = area / ab.magnitude()
        return cd

    else:
        return min(distance(l.p0, p), distance(l.p1, p))


def isclose(l1, l2, tolerance):
    if isinstance(l1, seg) and isinstance(l2, seg):
        if dist3D(l1, l2) < tolerance:
            return True
    elif isinstance(l1, seg) and isinstance(l2, vec):
        if DistancePtLine(l1, l2) < tolerance:
            return True
    elif isinstance(l1, vec) and isinstance(l2, seg):
        if DistancePtLine(l2, l1) < tolerance:
            return True
    elif isinstance(l1, vec) and isinstance(l2, vec):
        import math
        if math.sqrt((l1.x - l2.x) ** 2 + (l1.y - l2.y) ** 2) < tolerance:
            return True
    return False

def get_path_segs(p:list):
    path_segs = []
    if len(p) > 1:
        for i in range(1,len(p)):
            path_segs.append(seg(p[i-1],p[i]))
    else:
        path_segs = p
    return path_segs

def distance(u: vec, v: vec):
    import math
    return math.sqrt(vec(u.x - v.x, u.y - v.y, u.z - v.z).magnitude())


def path_is_close(l1, l2, tolerance = 0.5):

    l1 = get_path_segs(l1)
    l2 = get_path_segs(l2)
    for path_seg_1 in l1:
        for path_seg_2 in l2:
            if isclose(path_seg_1, path_seg_2, tolerance):
                return True
    return False


def clear_path(paths, proposed_path, ignore = None):
    print("ignoring", ignore)
    for i in range(len(paths)):
        if paths[i] is not None and ignore is not i:
            if path_is_close(paths[i], proposed_path):
                print(paths[i], proposed_path, "are the conflicting paths")
                return False
    return True

def to_path(myList):
    returnpath = []
    for point in myList:
        returnpath.append(vec(point[0], point[1], 0))
    return returnpath
'''
a = RRT()
a.plan([0,0,0],[1,1,0])
c = a.Planning()
d = [vec(0.5,0.5,0)]
t = path_is_close(c,d,-1)
print(t)
'''