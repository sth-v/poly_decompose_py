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


def rot_matrix_affine(theta):
    rot = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta), np.cos(theta), 0.0]

    ])
    return rot


def rotate2d(vec, theta):
    return rot_matrix(theta) @ np.asarray(vec)


def basis_to_custom_canonical_form(vecs):
    x_like = [0, 3, 4, 7]  # y_like = [1, 2, 5, 6]

    l, th, thr = [], [], []

    for vec in vecs[..., :2]:

        ang = angle(np.array([1, 0]), np.asarray(vec))
        aa, bb = divmod(ang, pi / 4)

        if aa in x_like:
            vec_x = vec * (-1) if vec[0] < 0.0 else vec
            l.append([vec_x, rotate2d(vec_x, theta=0.5 * pi)])
            th_ = theta(vec_x)
            th.append(th_)
            thr.append(rot_matrix_affine(th_))
        else:
            vec_y = vec * (-1) if vec[1] < 0.0 else vec
            l.append([rotate2d(vec_y, theta=-0.5 * pi), vec_y])
            th_ = theta(rotate2d(vec_y, theta=-0.5 * pi))
            th.append(th_)
            thr.append(rot_matrix_affine(th_))
    return np.asarray(l), np.asarray(th), np.asarray(thr)


vec_l = [[40.27832240751013, 17.184793210588396], [-90.76909511670237, 342.689586349763],[-597.2236579991877, -170.50443049520254],
 [-302.8780582649051, -317.03347889892757],[308.1424075482064, -308.39204586390406],[204.62842559075216, 203.57871970254928],
 [278.06133277650224, 43.473096154630184], [101.89771133306203, -376.15734403952956],[63.9330358550651, 17.928993717767298],
 [9.30055009451462, -33.0232094200328], [65.70811673195567, 18.42902405373752], [-62.45231896225596, 221.67703934758902],
 [72.29789360892028, 20.2681416394189],[-90.92466560343746, 319.88111454155296]]

l, th, thr = basis_to_custom_canonical_form(np.asarray(vec_l))
print(thr)
