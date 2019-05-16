
import math,random,copy,point
import numpy as np
import matplotlib.pyplot as plt


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    #  print(np.rad2deg(t), p, np.rad2deg(q))

    return t, p, q, mode


def RSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)

    return t, p, q, mode


def LSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    return t, p, q, mode


def RSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode


def RLR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q, mode


def LRL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(ex, ey, eyaw, c):
    # nomalize
    dx = ex
    dy = ey
    D = math.sqrt(dx ** 2.0 + dy ** 2.0)
    d = D * c
    #  print(dx, dy, D, d)

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(- theta)
    beta = mod2pi(eyaw - theta)
    #  print(theta, alpha, beta, d)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    bcost = float("inf")
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    #  print(bmode)
    px, py, pyaw = generate_course([bt, bp, bq], bmode, c)

    return px, py, pyaw, bmode, bcost


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c):
    """
    Dubins path plannner
    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]
    output:
        px
        py
        pyaw
        mode
    """

    ex = ex - sx
    ey = ey - sy

    lex = math.cos(syaw) * ex + math.sin(syaw) * ey
    ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
    leyaw = eyaw - syaw

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c)

    px = [math.cos(-syaw) * x + math.sin(-syaw)
          * y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw)
          * y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]

    return px, py, pyaw, mode, clen

def generate_course(length, mode, c):

    px = [0.0]
    py = [0.0]
    pyaw = [0.0]

    for m, l in zip(mode, length):
        pd = 0.0
        if m == "S":
            d = 1.0 * c
        else:  # turning couse
            d = np.deg2rad(3.0)

        while pd < abs(l - d):
            #  print(pd, l)
            px.append(px[-1] + d / c * math.cos(pyaw[-1]))
            py.append(py[-1] + d / c * math.sin(pyaw[-1]))

            if m == "L":  # left turn
                pyaw.append(pyaw[-1] + d)
            elif m == "S":  # Straight
                pyaw.append(pyaw[-1])
            elif m == "R":  # right turn
                pyaw.append(pyaw[-1] - d)
            pd += d

        d = l - pd
        px.append(px[-1] + d / c * math.cos(pyaw[-1]))
        py.append(py[-1] + d / c * math.sin(pyaw[-1]))

        if m == "L":  # left turn
            pyaw.append(pyaw[-1] + d)
        elif m == "S":  # Straight
            pyaw.append(pyaw[-1])
        elif m == "R":  # right turn
            pyaw.append(pyaw[-1] - d)
        pd += d

    return px, py, pyaw

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=100):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """

        self.start = Node(start.x, start.y, start.yaw)
        self.end = Node(goal.x, goal.y, goal.yaw)
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=False):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            newNode = self.steer(rnd, nind)
            if self.__CollisionCheck(newNode, self.obstacleList):
                self.nodeList.append(newNode)


        # generate coruse
        lastIndex = self.get_best_last_index()
        #  print(lastIndex)

        if lastIndex is None:
            return None

        path = self.gen_final_course(lastIndex)
        return path

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def steer(self, rnd, nind):
        #  print(rnd)
        curvature = 0.5

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = dubins_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd[0], rnd[1], rnd[2], curvature)

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += clen
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        return rnd

    def get_best_last_index(self):
        #  print("get_best_last_index")

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= 0.1]
        if goalinds == []:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [point.point(self.end.x, self.end.y, self.end.yaw)]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append(point.point(ix, iy))
            #path.append([node.x, node.y])
            goalind = node.parent
        path.append(point.point(self.start.x, self.start.y,self.start.yaw))
        path.reverse()
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])


    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2
                 + (node.y - rnd[1]) ** 2
                 + (node.yaw - rnd[2] ** 2) for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        #make obstaclelist rectangular.
        for (ox1, oy1, l, b) in obstacleList:
            for (ix, iy) in zip(node.path_x, node.path_y):
                if ox1 <= ix  and ix <= ox1 + l  and oy1 <= iy  and iy <= oy1 + b  :
                    return False


                '''
                dx = ox - ix
                dy = oy - iy

                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision
                '''


        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


def getsegments(path1):
    segments = []
    if len(path1) is 1:
        return path1
    for p in range(len(path1)-1):
        segment = (path1[p],path1[p+1])
        segments.append(segment)
    return segments

def ccw(p1,p2,p3):
    return (p3.y-p1.y) * (p2.x-p1.x) > (p2.y-p1.y) * (p3.x-p1.x)


def segments_distance(s1,s2):
  """ distance between two segments in the plane:
      one segment is (x11, y11) to (x12, y12)
      the other is   (x21, y21) to (x22, y22)
  """

  x11 = s1[0].x
  x12 = s1[1].x
  x21 = s2[0].x
  x22 = s2[1].x
  y11 = s1[0].y
  y12 = s1[1].y
  y21 = s2[0].y
  y22 = s2[1].y

  if segments_intersect(x11, y11, x12, y12, x21, y21, x22, y22): return 0
  # try each of the 4 vertices w/the other segment
  distances = []
  distances.append(point_segment_distance(x11, y11, x21, y21, x22, y22))
  distances.append(point_segment_distance(x12, y12, x21, y21, x22, y22))
  distances.append(point_segment_distance(x21, y21, x11, y11, x12, y12))
  distances.append(point_segment_distance(x22, y22, x11, y11, x12, y12))

  return min(distances)

def segments_intersect(x11, y11, x12, y12, x21, y21, x22, y22):
  """ whether two segments in the plane intersect:
      one segment is (x11, y11) to (x12, y12)
      the other is   (x21, y21) to (x22, y22)
  """
  dx1 = x12 - x11
  dy1 = y12 - y11
  dx2 = x22 - x21
  dy2 = y22 - y21
  delta = dx2 * dy1 - dy2 * dx1
  if delta == 0: return False  # parallel segments
  s = (dx1 * (y21 - y11) + dy1 * (x11 - x21)) / delta
  t = (dx2 * (y11 - y21) + dy2 * (x21 - x11)) / (-delta)
  return (0 <= s <= 1) and (0 <= t <= 1)

import math

def point_segment_distance(px, py, x1, y1, x2, y2):
  dx = x2 - x1
  dy = y2 - y1
  if dx == dy == 0:  # the segment's just a point
    return math.hypot(px - x1, py - y1)

  # Calculate the t that minimizes the distance.
  t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

  # See if this represents one of the segment's
  # end points or a point in the middle.
  if t < 0:
    dx = px - x1
    dy = py - y1
  elif t > 1:
    dx = px - x2
    dy = py - y2
  else:
    near_x = x1 + t * dx
    near_y = y1 + t * dy
    dx = px - near_x
    dy = py - near_y

  return math.hypot(dx, dy)

# Return true if line segments AB and CD intersect
def intersect(p1,p2,p3,p4):
    return ccw(p1,p3,p4) != ccw(p2,p3,p4) and ccw(p1,p2,p3) != ccw(p1,p2,p4)

def segintersect(s1,s2):

    return intersect(s1[0],s1[1],s2[0],s2[1])

def pathintersect(p1,p2):
    seg1 = getsegments(p1)
    seg2 = getsegments(p2)
    if seg1 is [] or seg2 is []:
        return False
    for s1 in seg1:

        for s2 in seg2:
            if segintersect(s1,s2):
                return True
    return False

def pathdistance(p1,p2,mindistance):
    seg1 = getsegments(p1)
    seg2 = getsegments(p2)
    if seg1 is [] or seg2 is []:
        return False
    for s1 in seg1:
        for s2 in seg2:
            if segments_distance(s1, s2) <= mindistance:
                return True
    return False

'''
def main():
    print("Start " + __file__)
    # ====Search Path with RRT====
    obstacleList = [
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = point.point(0.0, 0.0, np.deg2rad(0.0))
    goal = point.point(10.0, 10.0, np.deg2rad(0.0))

    start1 = point.point(5.0,0.0,np.deg2rad(0.0))
    goal1 = point.point(5.0,10.0,np.deg2rad(0.0))

    path = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList).Planning(animation=False)
    path1 = RRT(start1, goal1, randArea=[-2.0, 15.0], obstacleList=obstacleList).Planning(animation=False)

    a = pathintersect(path,path1)
    print(a)

if __name__ == '__main__':
    main()
'''