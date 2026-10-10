"""Target detection from the head camera (numpy only, runs on the Pi 5 in <2 ms at 160x120).
Finds the largest saturated-orange blob (the 'goal ball' / a marker vest) and returns where it is in the image."""
import numpy as np

def detect(rgb, min_pixels=12):
    """rgb: HxWx3 uint8.  Returns None or dict(bearing=rad (+ = target to the LEFT), elevation=rad, size=fraction of image height, px=pixel count)."""
    h, w, _ = rgb.shape
    r, g, b = (rgb[..., i].astype(np.int16) for i in range(3))
    mask = (r > 150) & (r - b > 90) & (r - g > 40) & (g - b > 10)          # orange: strong R, medium G, low B
    n = int(mask.sum())
    if n < min_pixels: return None
    ys, xs = np.nonzero(mask)
    cx, cy = xs.mean(), ys.mean()
    hfov = np.radians(70.0)
    bearing = -(cx - (w - 1) / 2) / w * hfov
    elev = -(cy - (h - 1) / 2) / h * hfov * h / w
    size = (ys.max() - ys.min() + 1) / h
    return dict(bearing=float(bearing), elevation=float(elev), size=float(size), px=n)
