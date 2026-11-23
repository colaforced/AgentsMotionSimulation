import math

import numpy as np


class Polygon:
    """目标正多边形。

    除顶点坐标外，还把边界参数化为弧长 s ∈ [0, L)：从顶点 0 出发沿边界走过的距离。
    原版 plgsim.py 用 0.01 间隔的"齿链"节点索引表示边界上的位置，这里换成连续弧长，
    求"前后邻居的中点"同样只需对 s 做加减，且不受离散精度限制。
    """

    def __init__(self, m, r, h, cx=50.0, cy=50.0):
        if m < 3:
            raise ValueError("多边形边数 m 至少为 3")
        self.m, self.r, self.h, self.cx, self.cy = m, r, h, cx, cy
        self.vertexs = np.array([[cx + r * math.sin(2 * math.pi * i / m),
                                  cy + r * math.cos(2 * math.pi * i / m)] for i in range(1, m + 1)])
        self._edges = np.roll(self.vertexs, -1, axis=0) - self.vertexs  # 第 i 条边：顶点 i → 顶点 i+1
        self.edge_len = float(np.linalg.norm(self._edges[0]))  # 正多边形各边等长
        self.perimeter = self.edge_len * m

    def vertex_s(self, k):
        """顶点 k 的弧长坐标。"""
        return k * self.edge_len

    def point_at(self, s):
        """弧长 s 处的边界点 (x, y)。"""
        s %= self.perimeter
        i = min(int(s // self.edge_len), self.m - 1)
        t = (s - i * self.edge_len) / self.edge_len
        return self.vertexs[i] + t * self._edges[i]

    def project(self, p):
        """点 p 在边界上最近点的弧长坐标。"""
        d = np.asarray(p, dtype=float) - self.vertexs
        t = np.clip(np.einsum('ij,ij->i', d, self._edges) / self.edge_len ** 2, 0.0, 1.0)
        dist = np.linalg.norm(d - t[:, None] * self._edges, axis=1)
        i = int(np.argmin(dist))
        return ((i + t[i]) * self.edge_len) % self.perimeter

    def vertices_by_distance(self, p):
        """按到点 p 的距离从近到远排列的顶点下标。"""
        d = np.linalg.norm(self.vertexs - np.asarray(p, dtype=float), axis=1)
        return [int(k) for k in np.argsort(d, kind='stable')]

    def queue_capacity(self, n):
        """每个顶点的占领者最多分出的排队序号数 ceil((n-m)/m)，保证 m 个顶点容得下 n 个智能体。"""
        return max(0, math.ceil((n - self.m) / self.m))

    def queue_slot(self, k, order, n):
        """顶点 k 的第 order（从 1 开始）个排队位置。

        奇数号排向下一个顶点、偶数号排向上一个顶点，间距为 边长/(容量+1)。
        一条边上，两端顶点各自排出的位置合计不超过 容量 个，不会重叠。
        """
        gap = self.edge_len / (self.queue_capacity(n) + 1)
        steps = (order + 1) // 2
        sign = 1 if order % 2 == 1 else -1
        return self.point_at(self.vertex_s(k) + sign * steps * gap)
