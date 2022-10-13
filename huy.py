from shapely.geometry import MultiLineString, LineString
import math


class BaseSection:
    def __init__(self, angle, **kwargs):
        self.angle = angle
        self.an_normalized = self.angle_normalized()
        self.types = kwargs
        self.width_dep = self.width_dependency()

    def angle_normalized(self):
        if self.angle > math.pi / 4:
            if self.angle < math.pi / 2:
                ang = (math.pi / 2) - self.angle
            elif 3 * math.pi / 4 > self.angle > math.pi / 2:
                ang = self.angle - (math.pi / 2)
            else:
                ang = math.pi - self.angle
        else:
            ang = self.angle
        return ang

    def width_dependency(self):
        width = {}
        for key, value in self.types.items():
            new_w = value[0] + (((value[1] - value[0]) * self.an_normalized) / 0.872665)  # 50 degrees
            width[key] = round(new_w)
        return width


class PolyClusters:
    def __init__(self, coordinates, angle, **kwargs):
        self.coordinates = coordinates
        self.angle = angle
        self.line = MultiLineString([LineString(c) for c in coordinates])
        self.boundrec = list(self.line.minimum_rotated_rectangle.exterior.coords)
        self.length = self.seg_length()
        self.basesec = BaseSection(angle, **kwargs)
        self.sum = self.grid_options()

    # dolzhno bit opisano ranee, pomoika
    def seg_length(self):
        l = LineString([self.boundrec[0], self.boundrec[1]]).length
        return round(l)

    def grid_options(self):

        def combination_sum():
            ans = []
            temp = []
            arr = sorted(list(set(self.basesec.width_dep.values())))
            arr_vals = list(self.basesec.width_dep.values())
            arr_keys = list(self.basesec.width_dep.keys())
            ln = self.length
            find_numbers(ans, arr, temp, ln, 0, arr_vals, arr_keys)
            return ans

        def find_numbers(ans, arr, temp, ln, index, arr_vals, arr_keys):
            if arr[0] / 3 > ln >= 0:
                ans.append(sum(list(temp)) / 31)
            for i in range(index, len(arr)):
                if (ln - arr[i]) >= 0:
                    temp.append(int(arr_keys[arr_vals.index(arr[i])]) * 4.6)
                    find_numbers(ans, arr, temp, ln - arr[i], i, arr_vals, arr_keys)
                    temp.remove(int(arr_keys[arr_vals.index(arr[i])]) * 4.6)

        return combination_sum()
