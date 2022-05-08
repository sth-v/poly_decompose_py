from itertools import cycle, islice
from shapely.geometry import LineString
import numpy as np
from collections import deque
from operator import neg


def points_to_point_chain(points, length=3):
    """
    EN
    Gets an ordered list of vectors(points) with as an array with dimension (l, v),
    where l is the length of the sequence, v is the length of the vector itself.

    Returns an array of chains of vectors with dimension (l, n, v), where n is the chain element length

    RU
    Принимает упорядоченный список векторов(точек) с в виде массива с размерностью (l, v),
    где l - длина последовательности, v - длина самого вектора.

    Возвращает массив цепочек векторов с размерностью (l, n, v), где n - длина элемента цепи

    :param points: array
    :param length: int
    :return: ndarray

    >>> a = np.arange(36).reshape(12, 3)
    >>> a.shape
    Out[32]: (12, 3)
    >>> a
    Out[33]:
    array([[ 0,  1,  2],
           [ 3,  4,  5],
           [ 6,  7,  8],
           [ 9, 10, 11],
           [12, 13, 14],
           [15, 16, 17],
           [18, 19, 20],
           [21, 22, 23],
           [24, 25, 26],
           [27, 28, 29],
           [30, 31, 32],
    >>> point_chain = points_to_point_chain(a, length=4)
    >>> point_chain.shape
    Out[37]: (12, 4, 3)

    """
    zip_list = [points]
    for i in range(length - 1):
        dq = deque(points)
        dq.rotate(neg(i + 1))
        zip_list.append(list(dq))
    return np.asarray(list(zip(*zip_list)))

def is_convex(points_list, polygon):
    vals = list(islice(cycle(range(len(points_list))), 0, len(points_list) + 2))
    concave = []
    for i in range(len(points_list)):
        line = LineString([points_list[vals[i]], points_list[vals[i + 2]]])
        if polygon.covers(line):
            pass
        else:
            concave.append(points_list[vals[i + 1]])
    return concave


def vector_search(point_list, key):
    cr = []
    n = list(islice(cycle(range(len(point_list))), 0, len(point_list) + 2))
    for i in range(len(point_list)):
        v_one = [point_list[n[i]][0] - point_list[n[i + 1]][0], point_list[n[i]][1] - point_list[n[i + 1]][1], 0]
        v_two = [point_list[n[i + 1]][0] - point_list[n[i + 2]][0], point_list[n[i + 1]][1] - point_list[n[i + 2]][1],
                 0]
        v_o_norm, v_t_norm = v_one / np.linalg.norm(v_one), v_two / np.linalg.norm(v_two)
        if np.cross(v_o_norm, v_t_norm)[2] < key:
            cr.append(n[i + 1])
        else:
            pass
    cross = [*cr, cr[0] + cr[-1] + 1]
    double_points = [*point_list, *point_list]
    segments = [double_points[cross[c]:cross[c + 1] + 1] for c in range(len(cross) - 1)]
    return segments

