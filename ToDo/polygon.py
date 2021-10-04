import numpy as np


class Polygon:
    def __init__(self, m, r, h, cx = 50, cy = 50) -> object:
        self.m = m
        self.r = r
        self.h = h
        self.cx = cx
        self.cy = cy
        self.vertexs = []
        self.gen_ploygen_vertexs()
        self.isOccupied = [False for i in range(m)]

    def gen_ploygen_vertexs(self):
        for i in range(1, self.m + 1):
            x = self.cx + self.r * np.sin(i * 2 * np.pi / self.m)
            y = self.cy + self.r * np.cos(i * 2 * np.pi / self.m)
            self.vertexs.append([x, y])




