from shapely.geometry import MultiPoint
import numpy as np
from itertools import product, chain
import shapely
import random
from shapely.geometry import Polygon, Point

bounds = [[-368.795886, 374.061718], [-360.627251, 87.64359], [-412.819468, 28.560247], [-418.991724, -317.973435],
          [30.744717, -375.504963],
          [460.65629, -365.913077], [460.503455, 71.040541], [97.37011, 158.431257], [-165.661215, 177.590117],
          [-160.623469, 568.751626],
          [-226.388709, 569.004352], [-227.04209, 603.066681], [-294.582026, 603.410702], [-295.060543, 372.726072],
          [-368.795886, 374.061718]]


class GridOnPolygon:
    def __init__(self, split_points, cellsx, cellsy, amount, road=40):
        self.split_points = split_points
        self.cells_x = cellsx
        self.cells_y = cellsy
        self.amount = amount
        self.road = road
        self.bound = np.asarray(MultiPoint([i for i in self.split_points]).bounds).reshape((2, 2)).T
        self.rect = np.array(np.meshgrid(self.bound[0], self.bound[1])).T.reshape(-1, 2)
        self.points_proj = self.points_projected()
        self.dist_vals, self.dist_coors = self.distances()
        # self.x, self.y = self.coordinates_dist()

    def points_projected(self):
        axes = [[self.rect[0], self.rect[2]], [self.rect[0], self.rect[1]]]
        points = []
        for i, v in enumerate(axes):
            temp = []
            length = np.sum((v[0] - v[1]) ** 2)
            for sp in self.split_points:
                t = max(0, min(1, np.sum((np.asarray(sp) - v[0]) * (v[1] - v[0])) / length))
                projection = np.asarray(v[0] + t * (v[1] - v[0]))
                temp.append(projection)
            points.append(np.asarray(sorted(temp, key=lambda k: [k[1], k[0]])))
        return np.asarray(points)

    def distances(self):
        arr = np.delete(self.points_proj, -1, axis=1)
        shift_arr = np.delete(np.roll(self.points_proj, -1, axis=1), -1, axis=1)
        dist = np.linalg.norm(arr - shift_arr, axis=2)
        values = []
        distance = []

        for p, i in enumerate(dist):
            values_to_del = []
            d = []
            sum_v = 0
            for v in range(len(i)):
                if i[v] + sum_v < (min(self.cells_x)+road) and v != len(i) - 1:
                    sum_v += i[v]
                    values_to_del.append(v + 1)

                elif i[v] + sum_v < (min(self.cells_x)+road) and v == len(i) - 1:
                    sum_v += i[v]
                    d[-1] += sum_v
                    values_to_del.append(v)

                else:
                    d.append(sum + i[v])
                    sum = 0
            values.append(d)
            clear_list = np.delete(self.points_proj[p], values_to_del, axis=0)
            distance.append(
                np.stack((np.delete(clear_list, -1, axis=0), np.delete(np.roll(clear_list, -1, axis=0), -1, axis=0)),
                         axis=1))
        return values, np.asarray(distance)

    def find_numbers(self, ans, temp, dist, vals, index):
        if dist == 0:
            ans.append(list(temp))
        for i in range(index, len(vals)):
            if (dist - self.road - vals[i]) >= 0:
                temp.append([self.road // 2, vals[i], self.road // 2])
                self.find_numbers(ans, temp, (dist - self.road - vals[i]), vals, i)
                temp.remove([self.road // 2, vals[i], self.road // 2])

    def grid_options(self, d, vals):
        combination = []
        ans = []
        temp = []
        self.find_numbers(ans, temp, d, vals, 0)
        if len(ans) <= 25:
            for i in ans:
                combination.append([item for sublist in i for item in sublist])
        else:
            for i in random.sample(ans, 25):
                combination.append([item for sublist in i for item in sublist])

        return combination

    '''def coordinates_dist(self):
        fin_xx = []
        fin_yy = []
        for ii, vv in enumerate(self.dist_vals[0]):
            kk = []
            options = self.grid_options(round(vv), self.cells_x)
            print(options)
            vec_one = self.dist_coors[0][ii][1] - self.dist_coors[0][ii][0]
            for o in options:
                dist_l = 0
                opt = []
                for oo in o:
                    dist_l += oo
                    p_one = self.dist_coors[0][ii][0] + vec_one / np.linalg.norm(vec_one) * dist_l
                    opt.append(p_one[0].tolist())
                kk.append(opt)
            fin_xx.append(kk)
        fin_x = [list(chain(*i)) for i in list(product(*fin_xx))]


        for ii, vv in enumerate(self.dist_vals[1]):
            kk = []
            options = self.grid_options(round(vv), self.cells_y)
            print(options)
            vec_one = self.dist_coors[1][ii][1] - self.dist_coors[1][ii][0]
            for o in options:
                dist_l = 0
                opt = []
                for oo in o:
                    dist_l += oo
                    p_one = self.dist_coors[1][ii][0] + vec_one / np.linalg.norm(vec_one) * dist_l
                    opt.append(p_one[1].tolist())
                kk.append(opt)
            fin_yy.append(kk)
        fin_y = [list(chain(*i)) for i in list(product(*fin_yy))]

        return fin_x, fin_y

    def meshgrid(self):
        grid = []
        if len(self.x) <= len(self.y):
            for x_, y_ in zip(self.x, self.y[0:len(self.x)]):
                xx,yy = np.meshgrid(np.asarray(x_), np.asarray(y_), indexing='xy')
                grid.append(np.stack([xx,yy]).T)
        else:
            for x_, y_ in zip(self.x[0:len(self.y)], self.y):
                xx,yy = np.meshgrid(np.asarray(x_), np.asarray(y_), indexing='xy')
                grid.append(np.stack([xx,yy]).T)
        return grid'''


Cx = list(range(58, 100))
Cy = list(range(58, 100))
amount = 12
road = 40
b_bound = GridOnPolygon(bounds, Cx, Cy, amount, road)

n, m = b_bound.distances()


def grid_from_intersect(coord, vals):
    grid = []
    x = np.cumsum(np.insert(np.array(vals[0]), 0, coord[0][0][0][0]))
    x_mid = (x[0:len(x) - 1] + np.roll(x, -1)[0:len(x) - 1]) / 2
    y = np.cumsum(np.insert(np.array(vals[1]), 0, coord[0][0][0][1]))
    y_mid = (y[0:len(y) - 1] + np.roll(y, -1)[0:len(y) - 1]) / 2
    x__, y__ = np.meshgrid(x_mid, y_mid, indexing='xy')
    grid = np.stack([x__, y__]).T
    return np.asarray(grid)


def point_in_poly(coord, vals, poly):
    polygon = Polygon(poly)
    grid = grid_from_intersect(coord, vals)

    def point_in(d):
        res = polygon.contains(Point(d.tolist()))
        return res

    result = list(map(point_in, grid.reshape(1, len(grid) * len(grid[0]), 2)[0]))
    return ~np.asarray(result).reshape(len(grid), len(grid[0])).T


v = point_in_poly(m, n, bounds)
print(v.tolist())
