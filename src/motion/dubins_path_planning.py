"""

Dubins path planner sample code

author Atsushi Sakai (@Atsushi_twi)
modified by ritwika ghosh for cyphyhouse

"""
# TODO: Complete documentation

import math

#import matplotlib.pyplot as plt
import numpy as np

from src.motion.pos import Pos


def mod2pi(theta: float) -> float:
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def LSL(alpha: float, beta: float, d: float):
    """
    Left to straight to left
    :param alpha:
    :param beta:
    :param d:
    :return:
    """
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


def RSR(alpha: float, beta: float, d: float):
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


def LSR(alpha: float, beta: float, d: float):
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


def RSL(alpha: float, beta: float, d: float):
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


def RLR(alpha: float, beta: float, d: float):
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


def LRL(alpha: float, beta: float, d: float):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    #compute numbers from dynamics
    tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(ex: float, ey: float, eyaw: float, c: float):
    # nomalize
    dx = ex
    dy = ey
    D = math.sqrt(dx ** 2.0 + dy ** 2.0)
    d = D / c
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
            #  print("".join(mode) + " cannot generate path")
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    #  print(bmode)
    px, py, pyaw, path1 = generate_course([bt, bp, bq], bmode, c)

    return px, py, pyaw, path1, bmode, bcost


def dubins_path_planning(sx: float, sy: float, syaw: float, ex: float, ey: float, eyaw: float, c: float):
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

    lpx, lpy, lpyaw, path1, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c)

    px = [math.cos(-syaw) * x + math.sin(-syaw)
          * y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw)
          * y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]
    path2 = [Pos(np.array([px[i], py[i], 0.0])) for i in range(len(px))]

    #  print(syaw)
    #  pyaw = lpyaw

    #  plt.plot(pyaw, "-r")
    #  plt.plot(lpyaw, "-b")
    #  plt.plot(eyaw, "*r")
    #  plt.plot(syaw, "*b")
    #  plt.show()

    return px, py, pyaw, path2, mode, clen


def generate_course(length: list, mode: str, c: float):
    px = [0.0]
    py = [0.0]
    pyaw = [0.0]
    path1 = [Pos(np.array([px[0], py[0], 0.0]))]

    for m, l in zip(mode, length):
        pd = 0.0
        if m == "S":
            d = 1.0 / c
        else:  # turning course
            d = np.deg2rad(3.0)

        while pd < abs(l - d):
            #  print(pd, l)
            px.append(px[-1] + d * c * math.cos(pyaw[-1]))
            py.append(py[-1] + d * c * math.sin(pyaw[-1]))
            path1.append(Pos(np.array([px[-1], py[-1], 0.0])))

            if m == "L":  # left turn
                pyaw.append(pyaw[-1] + d)
            elif m == "S":  # Straight
                pyaw.append(pyaw[-1])
            elif m == "R":  # right turn
                pyaw.append(pyaw[-1] - d)
            pd += d

        d = l - pd
        px.append(px[-1] + d * c * math.cos(pyaw[-1]))
        py.append(py[-1] + d * c * math.sin(pyaw[-1]))
        path1.append(Pos(np.array([px[-1], py[-1], 0.0])))

        if m == "L":  # left turn
            pyaw.append(pyaw[-1] + d)
        elif m == "S":  # Straight
            pyaw.append(pyaw[-1])
        elif m == "R":  # right turn
            pyaw.append(pyaw[-1] - d)
        pd += d

    return px, py, pyaw, path1


'''

if __name__ == '__main__':
    print("Dubins path planner sample start!!")

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 1.0

    px, py, pyaw, path2, mode, clen = dubins_path_planning(start_x, start_y, start_yaw,
                                                    end_x, end_y, end_yaw, curvature)

    plt.plot(px, py, label="final course " + "".join(mode))

    # plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)

    #  for (ix, iy, iyaw) in zip(px, py, pyaw):
    #  plot_arrow(ix, iy, iyaw, fc="b")
    print(path2)

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
'''