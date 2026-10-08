"""Observed obstacle and free-cell memory; all unseen cells remain unknown."""
import math
from .types import Config


def ray_cells(a, b):
    """Integer grid ray including both endpoints (Bresenham)."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx - dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            break
        twice = 2*error
        if twice > -dy:
            error -= dy
            x0 += sx
        if twice < dx:
            error += dx
            y0 += sy


class ObstacleMemory:
    def __init__(self, cfg: Config, ttl=3.0, free_ttl=30.0):
        if ttl <= 0 or free_ttl <= 0:
            raise ValueError('positive observation TTL required')
        self.cfg, self.ttl, self.free_ttl = cfg, ttl, free_ttl
        self.cells = {}  # cell -> timestamp of most recent hit
        self.free = {}   # cell -> timestamp observed clear along an actual beam

    def cell(self, xy):
        return int(xy[0] / self.cfg.cell), int(xy[1] / self.cfg.cell)

    def update(self, origin, points, now):
        if not all(math.isfinite(v) for v in (*origin, now)):
            return
        source = self.cell(origin)
        # Clear old returns along measured beams, then mark *all* endpoints.
        # A distant return must not erase a nearer return in this same frame.
        endpoints = []
        for p in points:
            if not all(math.isfinite(v) for v in p):
                continue
            if not (0 <= p[0] < self.cfg.width and 0 <= p[1] < self.cfg.height):
                continue
            endpoints.append(self.cell(p))
        current_hits = set(endpoints)
        for end in endpoints:
            for cell in ray_cells(source, end):
                if cell not in current_hits:
                    self.cells.pop(cell, None)
                    self.free[cell] = now
        for end in current_hits:
            self.cells[end] = now
            self.free.pop(end, None)
        self.prune(now)

    def prune(self, now):
        self.cells = {key: t for key, t in self.cells.items()
                      if 0 <= now-t <= self.ttl}
        self.free = {key: t for key, t in self.free.items()
                     if 0 <= now-t <= self.free_ttl}

    def points(self, now):
        self.prune(now)
        return [((i+0.5)*self.cfg.cell, (j+0.5)*self.cfg.cell)
                for i, j in self.cells]

    def known_free(self, now):
        self.prune(now)
        return set(self.free).difference(self.cells)
