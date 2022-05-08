# This is a sample Python script.
import copy

from core import *
import json

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.

features = {
    "parallel": [[Vector2d(1.0, 0.0), Vector2d(1.0, 0.0)],
                 [Vector2d(0.0, 1.0), Vector2d(0.0, 1.0)],
                 [Vector2d(0.0, 1.0), Vector2d(0.0, -1.0)]],
    "square": [[Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)],
               [Vector2d(0.0, 1.0), Vector2d(1.0, 1.0)],
               [Vector2d(0.0, -1.0), Vector2d(1.0, 0.0)]],

    "unidirectional": [Vector2d(1.0, 0.0), Vector2d(1.0, 0.0)],
    "multidirectional": [Vector2d(1.0, 0.0), Vector2d(1.0, 0.0)],
}

f = open('simple_test_data.json', 'rb')
data = json.load(f)
f.close()
simple_test_poly = np.asarray(data['simple_test_poly'])


def make_vectors2d(arr):
    for a in arr:
        yield Vector2d(*a)


vc = list(make_vectors2d((simple_test_poly - np.roll(simple_test_poly, 1, axis=0))[..., :2]))


def metric_parall(a, b):
    angle = a.angle(b)
    return Vector2d(angle, (2 * math.pi) - angle).length


def metric_sq(a, b, targets=(Vector2d(1.0, 0.0), Vector2d(0.0, 1.0))):
    dist = a.distance(b)
    target_a, target_b = targets
    dist_t = target_a.distance(target_b)
    print(dist, dist_t)
    return (dist - dist_t) ** 2


def metric_edge(a, b):
    dist = a.distance(b)
    dst = np.nan_to_num(dist)
    dist_t = 0.0
    print(dst, dist_t)
    return (dst - dist_t) ** 2


def min_angle_pairs(vectors, metric):
    for i, vector in enumerate(vectors):
        angle_list = []
        for j, other in enumerate(vectors):
            angle_list.append([metric(vector, other), i, j])
        angle_list.sort(key=lambda x: x[0])
        yield angle_list


def slv(vectors, metric, **kwargs):
    z = []
    for i in range(vectors.shape[0]):
        z.append(
            metric(
                (
                    make_vecs(vectors)[i],
                    np.roll(
                        make_vecs(vectors),
                        -1,
                        axis=0)[i]
                ),
                **kwargs
            )
        )
    return np.asarray(z)


def sorter_m(data, metrics, stop):
    stops = [0]
    tree = []
    for i, m in enumerate(metrics):
        if m >= stop:
            tree.append(
                (
                    data[stops[-1]:i - 1]).tolist()
            )
            stops.append(i)
    return stops, tree

def sort_with_inul(vectors, gen, metric):
    sample_data = np.asarray(
        list(
            gen(
                vectors,
                metric)
        )
    )
    return sample_data


def make_vecs(pts):
    return pts - np.roll(pts, 1, axis=0)


def champion(vectors, **kwargs):
    l = list(sort_with_inul(vectors, **kwargs))
    l.sort(key=lambda x: x[2])
    return l


def arr_mm(cch):
    new_ = []
    cmp = []
    for i in range(cch.shape[1]):

        for j in range(cch.shape[2]):
            ass = cch[i, j, 1:]
            ass.sort()

            if ass[0] == ass[1]:

                pass
            elif complex(*ass) in cmp:
                print(complex(*ass))
                pass
            else:
                print(ass)
                new_.append(cch[i, j])
                cmp.append(complex(*ass))
    return np.asarray(new_)


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
