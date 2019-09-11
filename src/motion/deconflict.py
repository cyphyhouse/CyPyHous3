# File contains all the functions used in de-conflicting paths

from typing import Union

import numpy as np

import src.motion.pos_types as pos


def is_collinear(p1: pos.Pos, p2: pos.Pos, p3: pos.Pos) -> bool:
    """
    function to check whether three points are collinear
    :param p1: point 1
    :param p2: point 2
    :param p3: point 3
    :return: true if collinear, false otherwise
    """
    n1 = pos.Pos(np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p3.z]))
    n2 = pos.Pos(np.array([p3.x - p1.x, p3.y - p1.y, p3.z - p1.z]))
    n3 = pos.cross(n1, n2)
    if n3.magnitude() == 0:
        return True
    return False


def line_to_pt_dist(l: pos.Seg, p: pos.Pos) -> float:
    """
    function to compute distance of line segment to a point
    :param l: line segment
    :param p: point
    :return: float distance
    """
    if not is_collinear(l.start, l.end, p):
        ab = l.end - l.start
        ac = p - l.start
        area = pos.cross(ab, ac).magnitude()
        cd = area / ab.magnitude()
        return cd

    else:
        d1 = pos.distance(l.start, p)
        d2  = pos.distance(l.end, p)
        d3 = l.length()
        import math
        if math.isclose(d1 + d2, d3):
            return 0.0
        return min(pos.distance(l.start, p), pos.distance(l.end, p))


def seg_distance_3d(l1: pos.Seg, l2: pos.Seg, tol=0.000001) -> float:
    """
    distance between two line segments in 3d
    :param l1: line segment 1
    :param l2: line segment 2
    :param tol: parameter to avoid 0 division
    :return: float distance between segments
    """
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
    elif tN > tD:
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


def is_close(l1: Union[pos.Seg, pos.Pos], l2: Union[pos.Pos, pos.Seg], tolerance: float) -> bool:
    """
    function to define closeness of two objects, point, or segment
    :param l1: line or segment
    :param l2: line or segment
    :param tolerance: tolerance
    :return: boolean
    """
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


def get_path_segs(p: list) -> list:
    """
    return segments of a path formed by points
    :param p: list of points
    :return: list of segments
    """
    path_segs = []
    if len(p) > 1:
        for i in range(1, len(p)):
            path_segs.append(pos.Seg(p[i - 1], p[i]))
    else:
        path_segs = p
    return path_segs


def path_is_close(l1: list, l2: list, tolerance=0.5) -> bool:
    """
    function to check whether any two segments of given paths are close.
    :param l1:
    :param l2:
    :param tolerance:
    :return:
    """
    if (l1[-1].z > 0 and l2[-1].z == 0) or (l1[-1].z == 0 and l2[-1] > 0):
        return False
    l1 = get_path_segs(l1)
    l2 = get_path_segs(l2)
    for path_seg_1 in l1:
        for path_seg_2 in l2:
            if is_close(path_seg_1, path_seg_2, tolerance):
                return True
    return False


def clear_path(paths: list, proposed_path: list, ignore=None, tolerance=0.5) -> bool:
    """
    function to determine if a list of paths doesn't conflict with a proposed path
    :param paths: list of existing paths
    :param proposed_path: proposed path
    :param ignore: ignore path pids to deconflict
    :param tolerance: closeness parameter
    :return: true if clear, false otherwise
    """
    # print("ignoring", ignore)
    if proposed_path is None:
        return False
    for i in range(len(paths)):
        if paths[i] is not None and ignore is not i:
            if path_is_close(paths[i], proposed_path, tolerance):
                # print(paths[i], proposed_path, "are the conflicting paths")
                return False
    return True

