import random

import numpy as np

from .agent import Agent, Phase
from .polygon import Polygon


class Simulation:
    """生成智能体并异步调度：每一轮按随机顺序逐个激活，同一时刻只有一个个体在决策和移动。"""

    def __init__(self, n=10, m=5, r=30.0, h=200.0, cx=50.0, cy=50.0, area=100.0, min_dist=5.0, seed=None,
                 v_z=4.0, v_xy=1.0, stagger_span=350.0, uniform_tol=0.05):
        if n < 1:
            raise ValueError("智能体数量 n 至少为 1")
        if h - stagger_span * (n - 1) / (2 * n) <= 0:
            raise ValueError("最低高度层低于地面，请增大编队高度 h 或减小 stagger_span")
        self.area = area
        self.rng = random.Random(seed)
        self.pgn = Polygon(m, r, h, cx, cy)
        self.agents = []
        for x, y in self._spawn(n, area, min_dist):
            self.agents.append(Agent(x, y, 0.0, self.pgn, self.agents, self.rng, v_z=v_z, v_xy=v_xy,
                                     stagger_span=stagger_span, uniform_tol=uniform_tol))
        self._order = list(range(n))
        self.rounds = 0

    def _spawn(self, n, area, min_dist):
        """在 [0, area]² 内随机生成两两间距不小于 min_dist 的地面初始位置。"""
        pts = []
        for _ in range(10000 * n):
            p = (self.rng.uniform(0, area), self.rng.uniform(0, area))
            if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= min_dist ** 2 for q in pts):
                pts.append(p)
                if len(pts) == n:
                    return pts
        raise ValueError(f"无法在 {area}x{area} 区域内放下 {n} 个间距 ≥ {min_dist} 的智能体")

    @property
    def phase(self):
        """整体所处阶段（取最慢的个体）。"""
        return min(a.phase for a in self.agents)

    @property
    def done(self):
        return self.phase == Phase.DONE

    def positions(self):
        return np.array([a.pos for a in self.agents])

    def step(self):
        """仿真一轮：随机打乱顺序后依次激活每个个体一次。"""
        self.rng.shuffle(self._order)
        for i in self._order:
            self.agents[i].activate()
        self.rounds += 1

    def run(self, max_rounds=100000):
        """一直运行到编队完成或达到轮数上限，返回是否完成。"""
        while not self.done and self.rounds < max_rounds:
            self.step()
        return self.done
