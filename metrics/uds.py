from functools import reduce
from operator import add


def uds_metric(area, la, lb):
    ans = add(
        reduce(add, la) * 1e-3,
        (reduce(add, lb) * 1e-3) / 2) / (area * 1e-6)
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
