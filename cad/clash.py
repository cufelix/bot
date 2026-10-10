import itertools, cadquery as cq
def bb_overlap(a, b, m=0.0):
    A, B = a.BoundingBox(), b.BoundingBox()
    return not (A.xmax < B.xmin - m or B.xmax < A.xmin - m or A.ymax < B.ymin - m or B.ymax < A.ymin - m or A.zmax < B.zmin - m or B.zmax < A.zmin - m)
def vol(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return -1.0
def clashes(items, allow=(), tol=1.0):
    """items: dict name->Shape. allow: set of frozenset pairs to ignore. returns list of (a,b,vol)."""
    res = []
    for (na, a), (nb, b) in itertools.combinations(items.items(), 2):
        if frozenset((na, nb)) in allow or not bb_overlap(a, b): continue
        v = vol(a, b)
        if v > tol or v < 0: res.append((na, nb, round(v, 1)))
    return res
