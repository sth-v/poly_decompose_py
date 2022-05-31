from shapely.geometry import MultiPoint
import numpy as np
from itertools import product, chain
import random


bounds = [[-567.92818426945519, 205.88996628914117], [-557.82343157092316, -308.27260320317788], [-74.101119767263754, -304.36817975841376], [573.18474164892473, -253.6658965791683], [494.80897953203686, 201.53875055651079], [-55.109039246149862, 168.48478557309198]]


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
        self.x, self.y = self.coordinates_dist()

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
            sum = 0
            for v in range(len(i)):
                if i[v] + sum < 98 and v != len(i) - 1:
                    sum += i[v]
                    values_to_del.append(v + 1)

                elif i[v] + sum < 98 and v == len(i) - 1:
                    sum += i[v]
                    d[-1] += sum
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

    def coordinates_dist(self):
        fin_xx = []
        fin_yy = []
        for ii, vv in enumerate(self.dist_vals[0]):
            kk = []
            options = self.grid_options(round(vv), self.cells_x)
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
        return grid






Cx = list(range(58, 100))
Cy = list(range(58, 100))
amount = 12
road = 50
b_bound = GridOnPolygon(bounds, Cx, Cy, amount, road)

n = b_bound.meshgrid()
print(n)