import numpy as np


class Vector(np.ndarray):
    def __new__(cls, input_array):
        # Input array is an already formed ndarray instance
        # We first cast to be our class type

        if len(input_array) == 2:
            obj = np.insert(input_array, 2, 0.0, axis=0).view(cls)

        else:
            obj = np.asarray(input_array).view(cls)
        # Finally, we must return the newly created object:
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.unit = getattr(obj, 'unit', None)


class Vectors(Vector):
    def __new__(cls, input_array):
        # Input array is an already formed ndarray instance
        # We first cast to be our class type

        if len(input_array[input_array.shape[-1]]) == 2:
            obj = np.insert(input_array, input_array.shape[-1], 0.0, axis=len(input_array.shape) - 1).view(cls)
        else:
            obj = np.asarray(input_array).view(cls)
        return obj


class vector(np.ndarray):
    def __new__(cls, input_array):
        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        __a = np.asarray(input_array)
        if len(__a.shape) > 1:
            obj = Vectors(__a)
        else:
            obj = Vector(__a)
        # add the new attribute to the created instance
        obj.unit = (obj / np.linalg.norm(obj)).view(cls)
        # Finally, we must return the newly created object:
        return obj
