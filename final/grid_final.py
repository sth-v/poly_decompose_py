from shapely.geometry import MultiPoint
import numpy as np
from shapely.geometry import Polygon, Point

bounds = [[-368.795886, 374.061718], [-360.627251, 87.64359], [-412.819468, 28.560247], [-418.991724, -317.973435],
          [30.744717, -375.504963],
          [460.65629, -365.913077], [460.503455, 71.040541], [97.37011, 158.431257], [-165.661215, 177.590117],
          [-160.623469, 568.751626],
          [-226.388709, 569.004352], [-227.04209, 603.066681], [-294.582026, 603.410702], [-295.060543, 372.726072],
          [-368.795886, 374.061718]]
'''bounds = [[-517.420304, 269.309141],
[-16.209736, 191.231113],
[493.816093, 294.495602],
[632.341627, -184.047151],
[-13.69109, -343.981177],
[-625.722085, -128.636938]]'''



class GridOnPolygon:
    def __init__(self, split_points, cellsx, cellsy, road=40):
        self.split_points = split_points
        self.cells_x = cellsx
        self.cells_y = cellsy
        self.road = road
        self.bound = np.asarray(MultiPoint([i for i in self.split_points]).bounds).reshape((2, 2)).T
        self.rect = np.array(np.meshgrid(self.bound[0], self.bound[1])).T.reshape(-1, 2)
        self.points_proj, self.vec_x, self.vec_y = self.points_projected()
        self.dist_vals, self.dist_coors = self.distances()
        self.x, self.y = self.grid_options()

    # По сути подготовка к расчету ячеек - точки развернутого полигона проецируются на ректангл
    # и дальше между ними считаются расстояния и убиваются слишком короткие. Эту часть хорошо бы упростить

    def points_projected(self):
        axes = [[self.rect[0], self.rect[2]], [self.rect[0], self.rect[1]]]
        vec_x = self.rect[2] - self.rect[0]
        vec_y = self.rect[1] - self.rect[0]
        points = []
        for i, v in enumerate(axes):
            temp = []
            length = np.sum((v[0] - v[1]) ** 2)
            for sp in self.split_points:
                t = max(0, min(1, np.sum((np.asarray(sp) - v[0]) * (v[1] - v[0])) / length))
                projection = np.asarray(v[0] + t * (v[1] - v[0]))
                temp.append(projection)
            points.append(np.asarray(sorted(temp, key=lambda k: [k[1], k[0]])))
        return np.asarray(points), vec_x, vec_y

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
                if i[v] + sum_v < (min(self.cells_x) + road) and v != len(i) - 1:
                    sum_v += i[v]
                    values_to_del.append(v + 1)
                elif i[v] + sum_v < (min(self.cells_x) + road) and v == len(i) - 1:
                    sum_v += i[v]
                    d[-1] += sum_v
                    values_to_del.append(v)
                else:
                    d.append(sum_v + i[v])
                    sum_v = 0
            values.append(d)
            distance.append(np.delete(self.points_proj[p], values_to_del, axis=0).tolist())
        return values, distance

    # Основная функция, считающая комбинации ячеек

    def find_numbers(self, ans, temp, dist, vals, index):
        if dist == 0:
            if len(ans) < 1:
                #ans.append(np.cumsum(np.asarray([item for sublist in temp for item in sublist])).tolist())
                ans.append(np.cumsum(np.asarray(temp)).tolist())
            else:
                pass
        for i in range(index, len(vals)):
            if (dist - self.road - vals[i]) >= 0:
                temp.append([vals[i] + self.road])
                self.find_numbers(ans, temp, (dist - self.road - vals[i]), vals, i)
                temp.remove([vals[i] + self.road])

    def grid_gen(self, coords, comb, vect, ind):
        iter_two = []
        for c in comb:
            iter_one = []
            for cc in c:
                p_one = coords + vect / np.linalg.norm(vect) * cc
                iter_one.append(p_one[ind])
            iter_two.append([coords, *iter_one])
        return iter_two

    # Координаты и мешгрид

    def grid_options(self):
        combination_x = []
        combination_y = []
        for i, val in enumerate(self.dist_vals[0]):
            ans = []
            temp = []
            self.find_numbers(ans, temp, round(val), self.cells_x, 0)
            combination_x.append(self.grid_gen(self.dist_coors[0][i][0], ans, self.vec_x, 0))
        for i, val in enumerate(self.dist_vals[1]):
            ans = []
            temp = []
            self.find_numbers(ans, temp, round(val), self.cells_y, 0)
            combination_y.append(self.grid_gen(self.dist_coors[1][i][1], ans, self.vec_y, 1))
        return combination_x, combination_y

    def constr(self, ind):
        grid = []
        x_lists = [list(list(zip(*self.x))[ind])]
        y_lists = [list(list(zip(*self.y))[ind])]
        for xx, yy in zip(x_lists, y_lists):
            x_ = [item for sublist in xx for item in sublist]
            y_ = [item for sublist in yy for item in sublist]
            xxx, yyy = np.meshgrid(np.asarray(x_), np.asarray(y_), indexing='xy')
            grid.append(np.stack([xxx, yyy]).T.tolist())
        return grid

    def meshgrid(self):
        minLength_x = min(len(x) for x in self.x)
        minLength_y = min(len(y) for y in self.y)
        grids = []
        if minLength_x <= minLength_y:
            for i in range(0, minLength_x):
                gridding = self.constr(i)
                grids.append(gridding)
        else:
            for i in range(0, minLength_y):
                gridding_y = self.constr(i)
                grids.append(gridding_y)

        return grids



Cx = list(range(60, 160))
Cy = list(range(60, 161))
amount = 12
road = 40
b_bound = GridOnPolygon(bounds, Cx, Cy, road)
n, m = b_bound.distances()
optx= b_bound.meshgrid()
print(optx)

