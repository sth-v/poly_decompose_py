from operator import add
import numpy as np


def uds_metric(area, la, lb):
    ans = add(
        np.add.reduce(la) * 1e-3,
        (np.add.reduce(lb) * 1e-3) / 2) / (area * 1e-6)
    return ans


def estimate_uds_metric(target, area, **kwargs):
    diff = target - uds_metric(area, **kwargs)
    ans = {
        "difference": {
            "factor": diff,
            "real": diff * (area * 1e-6)
        }
    }
    return ans
