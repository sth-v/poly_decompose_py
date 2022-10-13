import torch
import numpy as np
from math import pi


def unit(vec):
    return vec / np.linalg.norm(vec)


def modx(vecs):
    return np.where(vecs[:, 0] < 0.0, vecs.T, vecs.T * (-1)).T


def mody(vecs):
    return np.where(vecs[:, 1] < 0.0, vecs.T, vecs.T * (-1)).T


def angle(A, B):
    return np.arccos(np.dot(unit(A), unit(B)))


def theta(xaxis):
    return angle(np.array([1, 0]), xaxis)


def rot_matrix(theta):
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]

    ])
    return rot


def rotate2d(vec, theta):
    return rot_matrix(theta) @ np.asarray(vec)


def xory(vecs):
    x_like = [0, 3, 4, 7]  # y_like = [1, 2, 5, 6]

    l, th = [], []

    for vec in vecs[..., :2]:

        ang = angle(np.array([1, 0]), np.asarray(vec))
        aa, bb = divmod(ang, pi / 4)

        if aa in x_like:
            vec_x = vec * (-1) if vec[0] < 0.0 else vec
            l.append([vec_x, rotate2d(vec_x, theta=0.5 * pi)])
            th.append(theta(vec_x))
        else:
            vec_y = vec * (-1) if vec[1] < 0.0 else vec
            l.append([rotate2d(vec_y, theta=-0.5 * pi), vec_y])
            th.append(theta(rotate2d(vec_y, theta=-0.5 * pi)))

    return np.asarray(l), np.asarray(th)
