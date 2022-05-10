from itertools import cycle, islice
from shapely.geometry import LineString, Polygon
import numpy as np
from collections import deque
from operator import neg
from basepolygon_class import GoalPolygon
from sklearn.cluster import AffinityPropagation
from sklearn import metrics
from scipy.cluster.hierarchy import ward, maxinconsts, fcluster
from scipy.spatial.distance import pdist
from itertools import combinations
from scipy.special import expit, logit

place = GoalPolygon('Жилой комплекс «Одинцово-1»')
v = place.polygon
s = v.simplify(10, preserve_topology=True)
coor_list = (list(s.boundary.coords[0:len(s.boundary.coords) - 1]))
poly = Polygon(s)


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
    double_points = [*range(len(point_list)), *range(len(point_list))]
    segments = [double_points[cross[c]:cross[c + 1]] for c in range(len(cross) - 1)]
    return segments

c = vector_search(coor_list, 0.7)
print(c)
def clusterize(poly_edges):
    lines = []
    vectors_list = []
    cross_list = []
    key_vals = vector_search(poly_edges.boundary.coords[0:len(s.boundary.coords) - 1], 0.7)
    print(key_vals)
    vertices = poly_edges.boundary.coords[0:len(s.boundary.coords)]
    for i in range(len(vertices) - 1):
        lines.append(LineString([vertices[i], vertices[i + 1]]))
    for i in key_vals:
        lengths = [lines[a].length for a in i]
        goal_val = i[lengths.index(max(lengths))]
        vect = [lines[goal_val].coords[0][0] - lines[goal_val].coords[1][0],
                lines[goal_val].coords[0][1] - lines[goal_val].coords[1][1]]
        vectors_list.append(list(vect / np.linalg.norm(vect)))
    for i in combinations(vectors_list, 2):
        cross_list.append(float(np.cross(i[0], i[1])))
    return cross_list


#v = clusterize(poly)

data = np.asarray([[0,0.5056470286775836, 0.12269896816725095, 0.9999081361724104, 0.9999939171214292, 0.9999993317127042],
        [0.5056470286775836, 0,0.6076836895225528, 0.855807499739078, 0.8609715349701016, 0.8621552924233207],
        [0.12269896816725095, 0.6076836895225528,0, 0.9940158642276464, 0.992865864172556, 0.9925851239501953],
        [0.9999081361724104, 0.855807499739078, 0.9940158642276464, 0,0.010066604913686106, 0.012398300333692436],
        [0.9999939171214292, 0.8609715349701016, 0.992865864172556, 0.010066604913686106,0, 0.002331840941869334],
        [0.9999993317127042, 0.8621552924233207, 0.9925851239501953, 0.012398300333692436, 0.002331840941869334,0]])



