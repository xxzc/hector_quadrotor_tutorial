# -*- coding: utf-8 -*-
import numpy as np
import math

def angle(v):
    """
    >>> angle(np.array([0,1]))
    1.5707963267948966
    """
    return math.atan2(v[1], v[0])


def np2int(a):
    return tuple(a.astype(int))
if __name__ == '__main__':
    import doctest
    doctest.testmod()