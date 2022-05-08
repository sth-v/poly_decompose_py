from itertools import cycle, islice
from shapely.geometry import Polygon, LineString
import numpy as np
from basepolygon_class import GoalPolygon

place = GoalPolygon('Жилой комплекс «Одинцово-1»')
v = place.polygon
s = v.simplify(10, preserve_topology=True)
coor_list = (list(s.boundary.coords[0:len(s.boundary.coords)-1]))
poly = Polygon(s)


def isconvex(points_list, polygon):
    vals = list(islice(cycle(range(len(points_list))), 0, len(points_list) + 2))
    concave = []
    for i in range(len(points_list)):
        line = LineString([points_list[vals[i]], points_list[vals[i+2]]])
        if polygon.covers(line):
            pass
        else:
            concave.append(points_list[vals[i+1]])
    return concave


def vector_search(point_list, key):
    cr = []
    n = list(islice(cycle(range(len(point_list))), 0, len(point_list) + 2))
    for i in range(len(point_list)):
        v_one = [point_list[n[i]][0] - point_list[n[i + 1]][0], point_list[n[i]][1] - point_list[n[i + 1]][1], 0]
        v_two = [point_list[n[i + 1]][0] - point_list[n[i + 2]][0], point_list[n[i + 1]][1] - point_list[n[i + 2]][1], 0]
        v_o_norm, v_t_norm = v_one / np.linalg.norm(v_one), v_two / np.linalg.norm(v_two)
        if np.cross(v_o_norm, v_t_norm)[2] < key:
            cr.append(n[i + 1])
        else:
            pass
    cross = [*cr, cr[0]+cr[-1]+1]
    double_points = [*point_list, *point_list]
    segments = [double_points[cross[c]:cross[c+1]+1] for c in range(len(cross)-1)]
    return segments




bb = vector_search(coor_list, 0.7)
print(bb)