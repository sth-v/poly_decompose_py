from itertools import repeat

default_weights = {
    "base_point": 0.3,
    "centroid": 0.9
    "custom": 0.0
}


class Attractors:
    DEF_W = {

    def __init__(self):
        pass


def d(x, y, z):
    global default_weights
    nm = ['centroid', "base_point", "custom"]
    for i, t in enumerate([x, y, z]):
        yield repeat((i, default_weights[nm[i]]), len(x))


def update_w(inx, w, *args, **kwargs):
    l = list(d(*args, **kwargs))
    for k, v in l:
        for i in inx:
            if i == k:

    return
