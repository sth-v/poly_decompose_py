import numpy as np
import copy
from typing import Any, Union, Tuple, Sequence, MutableMapping
from collections import OrderedDict
import torch


class GraphMeta(type):
    primitive = []
    gathering = []

    special_bases = {
        np.ndarray: ['__new__', '__array_finalize__'],
    }

    def __new__(mcs, classname, bases, attrs: dict):

        # set basic types
        mcs.__metatype__(GraphMeta, classname, attrs)

        # create class from type (super())
        graph = type(classname, bases, attrs)

        return graph

    def __special_bases_check(cls, classname, bases, attrs):
        list_bases = list(bases)

        if len(bases) == 0:
            list_bases = [object, ]
            ans_tuple = (classname, tuple(list_bases), attrs)

        else:
            parent = bases[0]

            if parent in cls.special_bases:
                list_bases.remove(parent)

                new_class_name = f'_{classname}'
                new_attrs = {}
                for n in cls.special_bases[parent]:
                    try:
                        new_attrs |= {n: attrs.pop(n)}
                    except:
                        pass
                obj = type(classname.lower(), (parent,), new_attrs)
                attrs = {new_class_name.lower(): obj}
                if len(list_bases) == 0:
                    list_bases.append(object)
                ans_tuple = (classname, tuple(list_bases), attrs)
            else:
                ans_tuple = (classname, bases, attrs)

        return ans_tuple

    def __metatype__(cls, classname: str, attrs: dict) -> None:
        try:
            key = attrs['meta_type']
            if hasattr(cls, key):

                print(f"[Append] meta type {key} : {classname}")
                lst = getattr(cls, key).append(classname)

                setattr(
                    cls,
                    key,
                    lst
                )
            else:
                print(f"[Create] meta type {key} : {classname}")
                setattr(
                    cls,
                    key,
                    [classname]
                )
            print(getattr(cls, key))

        except:
            pass


class GraphModel(metaclass=GraphMeta):
    meta_type = 'gathering'


class Node(OrderedDict, metaclass=GraphMeta):
    meta_type = 'primitive'


class Edge(np.ndarray, metaclass=GraphMeta):
    meta_type = 'primitive'

    def __new__(cls, input_array):
        arr = np.asarray(input_array)

        obj = arr.view(cls)
        obj.s = np.array(arr.imag, dtype=np.float32).view(cls)
        obj.e = np.array(arr.real, dtype=np.float32)

        return obj

    def __array_finalize__(self, obj):
        self.t = np.asarray(obj)
        self.i = None

    def __getattr__(self, item):
        return getattr(self, item)


class Aj(np.ndarray, metaclass=GraphMeta):
    meta_type = 'gathering'

    def __new__(cls, input_array):

        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        l = len(input_array)

        isx = np.arange(l)
        jsx = np.roll(isx, -1)
        ij = np.stack([isx, jsx], axis=1)

        mask_a = np.diag(input_array)

        conn = np.zeros((l, l), dtype=np.int32)
        for n in ij:
            i, j = n
            conn[i, j], conn[j, i] = 1, 1

        edges = np.ones((l, l)) * -1
        _edges = np.ones((l, l), dtype=np.int32) * -1
        for n in ij:
            i, j = n

            edges[i, j] = i

            edges[j, i] = j

            _edges[i, j] = np.roll(i, -1)
            _edges[i, i] = i
            _edges[j, i] = j
        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        obj = np.stack([mask_a, conn], axis=2).view(cls)
        obj.ij = ij.view(cls)
        obj.flat_inx = np.arange(l * l).reshape((l * l)).view(cls)

        full_i, full_j = np.meshgrid(isx, isx, indexing='ij')
        obj.full_i, obj.full_j = full_i.view(cls), full_j.view(cls)
        #
        obj.full_ij = torch.complex(
            torch.tensor(
                obj.full_i,
                dtype=torch.float32
            ),
            torch.tensor(
                obj.full_j,
                dtype=torch.float32
            )
        ).detach().numpy()

        # Непроработанные атрибуты
        obj._edg_l = ij.view(cls)
        obj._edg_h = _edges.view(cls)

        # Cвязанные дополнительные объекты
        #
        obj.mask = np.asarray(input_array).view(cls)
        obj.mask_indx = np.roll(np.extract(obj.mask == 1, obj.ij[:, 1]), 1).view(cls)
        obj.mask_indx_wn = np.roll(np.where(obj.mask == 1, obj.ij[:, 1] + 1, 0), 1).view(cls)
        # Sparce tensor Layers
        obj.tconn = conn.view(cls)
        obj.tedges = edges.view(cls)
        obj.tmasked = mask_a.view(cls)
        _ = np.where(np.roll(obj.mask_indx, -1) == 0, len(obj.mask) - 1, np.roll(obj.mask_indx, -1))
        x, y = np.meshgrid(obj.mask_indx, _, indexing='ij')
        np.stack([x, y], axis=2)
        # Finally, we must return the newly created object:
        return obj

    def __array_finalize__(self, obj):

        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.__arr = obj

        # self.ma_edges = self.masked_a()

    def masked_a(self):
        m = np.copy(self.mask)
        arr = np.copy(self._edg_h)
        a, b = arr.shape

        for i in range(a):
            if m[i] == 1:
                arr[i, i] = -13
        return arr

    def abs_norm_mask_lb(self):
        m = self.masked_a()
        return np.extract(m == -13, self.full_ij)
