import numpy as np
from numpy import ndarray
from scipy.spatial import ConvexHull
from sklearn.cluster import KMeans, DBSCAN, OPTICS
from time import gmtime, strftime
import math


def shortest_distance(x1, y1, z1, a, b, c, d):
    d = abs((a * x1 + b * y1 + c * z1 + d))
    e = (math.sqrt(a * a + b * b + c * c))
    print("Perpendicular distance is", d / e)
    return d / e


def distance_e(a, b):
    return Vector(*(a.vector - b.vector)).length


class _Vector:
    __symbols = ['x',
                 'y',
                 'z',
                 'w',
                 'u',
                 'v',
                 's',
                 't']

    def __init__(self, *args):
        self.vector = np.array(args)
        for i, arg in enumerate(args):
            a, b = divmod(i, (len(self.__class__.__symbols)))
            setattr(self, self.__class__.__symbols[b] * (a + 1), arg)

    def __str__(self):
        return f'vec {self.vector}'

    def __repr__(self):
        return f'vec {self.vector}'

    def __matmul__(self, other):
        return np.dot(self.vector, other.vector)

    def __mul__(self, other):
        if other.__class__ == self.__class__:
            ans = self.vector * other.vector
        else:
            ans = self.vector * other
        return ans

    def __neg__(self):
        return self.__class__(*np.negative(self.vector))

    def angle(self, other):
        return np.arccos(self.vector @ other.vector)

    def __length(self):
        vl = 0
        for n in self.vector:
            vl += n ** 2
        return math.sqrt(vl)

    def distance(self, other):
        return _Vector(*(self.vector-other.vector)).__length()


class Vector(_Vector):
    def __init__(self, x, y):
        super().__init__(x, y)
        __unit = self.vector / np.linalg.norm(self.vector)
        self.unit = _Vector(*__unit)
        self._length = self.__length()

    def __matmul__(self, other):
        return self.unit @ other.unit

    def angle(self, other):
        return np.arccos(self.unit.angle(other.unit))

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, val):
        super().__init__(*list(map(lambda x: val * x, self.unit.vector)))



class Vector2d(Vector):
    def __init__(self, x, y):
        super().__init__(x, y)
        self.complex = complex(self.x, self.y)
        self.linear_integral = (self.unit.x * self.unit.x) / 2

def convexhull(points: ndarray):
    hull = ConvexHull(points)
    sorted = []
    for i in hull.vertices:
        sorted.append(points[i])
    return sorted


def minimum_bound_rectangle(points: ndarray):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.
    :param points: a nx2 matrix of coordinates
    :rval: a nx2 matrix of coordinates
    """

    pi2 = np.pi / 2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points) - 1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    # XXX both work
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles - pi2),
        np.cos(angles + pi2),
        np.cos(angles)]).T

    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r)

    return rval


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)


def intersectsAbove(verts, v, u):
    """
        Returns True if uv intersects the polygon defined by 'verts' above v.
        Assumes v is the index of a vertex in 'verts', and u is outside of the
        polygon.
    """
    n = len(verts)

    # Test if two adjacent vertices are on same side of line (implies
    # tangency)
    if ccw(u, verts[v], verts[(v - 1) % n]) == ccw(u, verts[v], verts[(v + 1) % n]):
        return False

    # Test if u and v are on same side of line from adjacent
    # vertices
    if ccw(verts[(v - 1) % n], verts[(v + 1) % n], u) == ccw(verts[(v - 1) % n], verts[(v + 1) % n], verts[v]):
        return u.y > verts[v].y
    else:
        return u.y < verts[v].y


def get_clusters(data: ndarray, labels):
    """
    :param data: The dataset
    :param labels: The label for each point in the dataset
    :return: List[np.ndarray]: A list of arrays where the elements of each array
    are data points belonging to the label at that ind
    """
    return [data[np.where(labels == i)] for i in range(np.amax(labels) + 1)]


def kmeans(points: ndarray, n_clusters: int, random_state=0, **kwargs):
    return KMeans(n_clusters, random_state=random_state, **kwargs).fit(points)
