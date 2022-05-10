import numpy as np


def unitize(a):
    return a / np.linalg.norm(a)


def angle(a, b):
    return np.arccos(a @ b)
