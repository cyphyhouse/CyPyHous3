# File contains all the functions used in de-conflicting paths

import numpy as np

import src.motion.pos as pos


def is_collinear(p1: pos.Pos, p2: pos.Pos, p3: pos.Pos):
    n1 = pos.Pos(np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p3.z]))
    n2 = pos.Pos(np.array([p3.x - p1.x, p3.y - p1.y, p3.z - p1.z]))
    n3 = pos.cross(n1, n2)
    if n3.magnitude() == 0:
        return True
    return False


def line_to_pt_dist(l: pos.Seg, p: pos.Pos):
    if not is_collinear(l.start, l.end, p):
        ab = l.end - l.start
        ac = p - l.start
        area = pos.cross(ab, ac).magnitude()
        cd = area / ab.magnitude()
        return cd

    else:
        return min(pos.distance(l.start, p), pos.distance(l.end, p))


def seg_distance_3d(l1: pos.Seg, l2: pos.Seg, tol=0.000001):
    u = l1.end - l1.start
    v = l2.end - l2.start
    w = l1.start - l2.start
    a = pos.dot(u, u)
    b = pos.dot(u, v)
    c = pos.dot(v, v)
    d = pos.dot(u, w)
    e = pos.dot(v, w)
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

    dP = (w + (pos.Pos(np.array([sc * u.x, sc * u.y, sc * u.z])) - pos.Pos(np.array([tc * v.x, tc * v.y, tc * v.z]))))
    return dP.magnitude()


def is_close(l1, l2, tolerance):
    if isinstance(l1, pos.Seg) and isinstance(l2, pos.Seg):
        if seg_distance_3d(l1, l2) < tolerance:
            return True
    elif isinstance(l1, pos.Seg) and isinstance(l2, pos.Pos):
        if line_to_pt_dist(l1, l2) < tolerance:
            return True
    elif isinstance(l1, pos.Pos) and isinstance(l2, pos.Seg):
        if line_to_pt_dist(l2, l1) < tolerance:
            return True
    elif isinstance(l1, pos.Pos) and isinstance(l2, pos.Pos):
        if pos.distance(l1, l2) < tolerance:
            return True
    return False


def get_path_segs(p: list):
    path_segs = []
    if len(p) > 1:
        for i in range(1, len(p)):
            path_segs.append(pos.Seg(p[i - 1], p[i]))
    else:
        path_segs = p
    return path_segs


def path_is_close(l1, l2, tolerance=0.5):
    l1 = get_path_segs(l1)
    l2 = get_path_segs(l2)
    for path_seg_1 in l1:
        for path_seg_2 in l2:
            if is_close(path_seg_1, path_seg_2, tolerance):
                return True
    return False


def clear_path(paths, proposed_path, ignore=None):
    print("ignoring", ignore)
    for i in range(len(paths)):
        if paths[i] is not None and ignore is not i:
            if path_is_close(paths[i], proposed_path):
                # print(paths[i], proposed_path, "are the conflicting paths")
                return False
    return True


def to_path(myList):
    return_path = []
    for point in myList:
        return_path.append(pos.Pos(np.array[(point[0], point[1], 0)]))
    return return_path
